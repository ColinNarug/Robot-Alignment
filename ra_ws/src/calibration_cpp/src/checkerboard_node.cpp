// This node is to use a checkerboard as the target object

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/tf2/LinearMath/Matrix3x3.h>
#include <tf2/tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct ActiveCameraInfo
{
  std::string serial;
  int width{0};
  int height{0};
  bool valid() const { return !serial.empty() && width > 0 && height > 0; }
};

static std::string infer_workspace_root_from_share(const std::string& share_dir)
{
  const std::string needle = "/install/";
  const auto pos = share_dir.find(needle);
  if (pos == std::string::npos) return "";
  return share_dir.substr(0, pos);
}

static std::vector<std::string> compute_candidate_config_dirs()
{
  std::vector<std::string> dirs;

  for (const auto& pkg : std::vector<std::string>{"uralignment_cpp", "calibration_cpp", "uralignment_py"})
  {
    try
    {
      const std::string share = ament_index_cpp::get_package_share_directory(pkg);
      dirs.push_back(share + "/config");
    }
    catch (...) {}
  }

  std::string ws_root;
  try
  {
    const std::string share = ament_index_cpp::get_package_share_directory("uralignment_cpp");
    ws_root = infer_workspace_root_from_share(share);
  }
  catch (...) {}

  if (!ws_root.empty())
  {
    dirs.push_back(ws_root + "/config");
    dirs.push_back(ws_root + "/config/intrinsics");
    dirs.push_back(ws_root + "/src/uralignment_cpp/config");
    dirs.push_back(ws_root + "/src/calibration_cpp/config");
    dirs.push_back(ws_root + "/src/uralignment_py/config");
    dirs.push_back(ws_root + "/install/uralignment_cpp/share/uralignment_cpp/config");
    dirs.push_back(ws_root + "/install/calibration_cpp/share/calibration_cpp/config");
    dirs.push_back(ws_root + "/install/uralignment_py/share/uralignment_py/config");
  }

  std::vector<std::string> uniq;
  for (const auto& d : dirs)
  {
    if (!d.empty() && std::find(uniq.begin(), uniq.end(), d) == uniq.end())
      uniq.push_back(d);
  }
  return uniq;
}

static std::string find_first_existing(const std::vector<std::string>& dirs, const std::string& filename)
{
  for (const auto& d : dirs)
  {
    const std::string p = d + "/" + filename;
    if (fs::exists(p)) return p;
  }
  return "";
}

static bool read_active_camera_yaml(const std::string& path, ActiveCameraInfo& out)
{
  cv::FileStorage fsr(path, cv::FileStorage::READ);
  if (!fsr.isOpened()) return false;

  fsr["serial"] >> out.serial;
  fsr["width"]  >> out.width;
  fsr["height"] >> out.height;
  return out.valid();
}

static bool read_intrinsics_yaml(const std::string& path, cv::Mat& K, cv::Mat& D)
{
  cv::FileStorage fsr(path, cv::FileStorage::READ);
  if (!fsr.isOpened()) return false;

  if (!fsr["camera_matrix"].empty()) fsr["camera_matrix"] >> K;
  if (K.empty() && !fsr["K"].empty()) fsr["K"] >> K;

  if (!fsr["distortion_coefficients"].empty()) fsr["distortion_coefficients"] >> D;
  if (D.empty() && !fsr["D"].empty()) fsr["D"] >> D;

  return (K.rows == 3 && K.cols == 3);
}

class CheckerboardObjectNode : public rclcpp::Node
{
public:
  CheckerboardObjectNode() : Node("checkerboard_node")
  {
    image_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cam_opts;
    cam_opts.callback_group = image_handling_;
    object_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(this->get_logger(), "Checkerboard object node initialized.");

    // Keep the same basic runtime parameters as the AprilTag node.
    width_ = declare_parameter<int>("width", 1920);
    height_ = declare_parameter<int>("height", 1080);
    fps_ = declare_parameter<int>("fps", 30);
    active_camera_filename_ = this->declare_parameter<std::string>("active_camera_filename", "active_camera.yaml");
    qos_depth_ = this->declare_parameter<int>("qos_depth", 1);
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort");
    preferred_encoding_ = this->declare_parameter<std::string>("preferred_encoding", "bgr8");

    // Checkerboard pose parameters.
    board_cols_ = this->declare_parameter<int>("board_cols", 24); // inner corners
    board_rows_ = this->declare_parameter<int>("board_rows", 17); // inner corners
    square_size_m_ = this->declare_parameter<double>("square_size_m", 0.0075);
    detect_scale_ = this->declare_parameter<double>("detect_scale", 0.35);
    use_sb_ = this->declare_parameter<bool>("use_findChessboardCornersSB", true);
    adaptive_thresh_ = this->declare_parameter<bool>("adaptive_thresh", true);
    normalize_image_ = this->declare_parameter<bool>("normalize_image", true);
    fast_check_ = this->declare_parameter<bool>("fast_check", false);
    refine_corners_ = this->declare_parameter<bool>("refine_corners", true);
    refine_win_ = this->declare_parameter<int>("refine_window", 11);
    refine_max_iters_ = this->declare_parameter<int>("refine_max_iters", 30);
    refine_eps_ = this->declare_parameter<double>("refine_eps", 0.1);

    if (board_cols_ <= 1 || board_rows_ <= 1)
      throw std::runtime_error("board_cols and board_rows must be > 1 (inner corners).");
    if (square_size_m_ <= 0.0)
      throw std::runtime_error("square_size_m must be > 0.");
    if (detect_scale_ <= 0.0 || detect_scale_ > 1.0)
      detect_scale_ = 1.0;

    // QoS
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
    qos.durability_volatile();
    if (qos_reliability_ == "reliable") qos.reliable();
    else qos.best_effort();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", qos,
      std::bind(&CheckerboardObjectNode::image_callback, this, std::placeholders::_1), cam_opts);

    cMo_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("cMo", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CheckerboardObjectNode::object_loop, this),
      object_handling_);

    initialization();
  }

private:
  rclcpp::CallbackGroup::SharedPtr image_handling_, object_handling_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::mutex img_mtx_;
  bool have_img_{false};

  std::string active_camera_filename_{"active_camera.yaml"};
  ActiveCameraInfo active_;
  std::string active_camera_path_;
  std::string intrinsics_path_;
  bool have_cam_model_{false};

  int width_{1920};
  int height_{1080};
  int fps_{30};
  int qos_depth_{1};
  std::string qos_reliability_{"best_effort"};
  std::string preferred_encoding_{"bgr8"};

  int board_cols_{24};
  int board_rows_{17};
  double square_size_m_{0.0075};
  double detect_scale_{0.35};
  bool use_sb_{true};
  bool adaptive_thresh_{true};
  bool normalize_image_{true};
  bool fast_check_{false};
  bool refine_corners_{true};
  int refine_win_{11};
  int refine_max_iters_{30};
  double refine_eps_{0.1};

  cv::Size board_size_;
  std::vector<cv::Point3f> board_object_points_;
  cv::Mat K_, D_;

  void initialization()
  {
    board_size_ = cv::Size(board_cols_, board_rows_);
    build_centered_board_object_points_();

    if (!load_active_camera_and_intrinsics_once())
      throw std::runtime_error("Failed to load active camera / intrinsics YAML at startup.");

    RCLCPP_INFO(
      this->get_logger(),
      "board=%dx%d inner corners | square_size_m=%.6f | detect_scale=%.2f",
      board_cols_, board_rows_, square_size_m_, detect_scale_);
    RCLCPP_INFO(
      this->get_logger(),
      "This node publishes cMo for the checkerboard center. Keep the board orientation consistent across captures.");
  }

  void build_centered_board_object_points_()
  {
    board_object_points_.clear();
    board_object_points_.reserve(static_cast<size_t>(board_cols_ * board_rows_));

    const double cx = 0.5 * static_cast<double>(board_cols_ - 1) * square_size_m_;
    const double cy = 0.5 * static_cast<double>(board_rows_ - 1) * square_size_m_;

    for (int r = 0; r < board_rows_; ++r)
    {
      for (int c = 0; c < board_cols_; ++c)
      {
        board_object_points_.emplace_back(
          static_cast<float>(c * square_size_m_ - cx),
          static_cast<float>(r * square_size_m_ - cy),
          0.0f);
      }
    }
  }

  bool load_active_camera_and_intrinsics_once()
  {
    const auto dirs = compute_candidate_config_dirs();

    const std::string ac_path = find_first_existing(dirs, active_camera_filename_);
    if (ac_path.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not find %s", active_camera_filename_.c_str());
      return false;
    }

    ActiveCameraInfo ac;
    if (!read_active_camera_yaml(ac_path, ac))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read %s", ac_path.c_str());
      return false;
    }

    const std::string intrinsics_name =
      ac.serial + "_" + std::to_string(ac.width) + "x" + std::to_string(ac.height) + "_intrinsics.yaml";
    const std::string K_path = find_first_existing(dirs, intrinsics_name);
    if (K_path.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not find intrinsics yaml: %s", intrinsics_name.c_str());
      return false;
    }

    cv::Mat K, D;
    if (!read_intrinsics_yaml(K_path, K, D))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read intrinsics yaml: %s", K_path.c_str());
      return false;
    }

    active_ = ac;
    active_camera_path_ = ac_path;
    intrinsics_path_ = K_path;
    K_ = K.clone();
    D_ = D.clone();
    have_cam_model_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Loaded active camera from %s | serial=%s | %dx%d",
                active_camera_path_.c_str(), active_.serial.c_str(), active_.width, active_.height);
    RCLCPP_INFO(this->get_logger(), "Loaded intrinsics from %s", intrinsics_path_.c_str());
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg) return;
    std::scoped_lock lk(img_mtx_);
    latest_image_ = msg;
    have_img_ = true;
  }

  void object_loop()
  {
    if (!have_cam_model_) return;

    sensor_msgs::msg::Image::ConstSharedPtr img;
    {
      std::scoped_lock lk(img_mtx_);
      if (!have_img_ || !latest_image_) return;
      img = latest_image_;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      const std::string enc = (preferred_encoding_ == "rgb8")
                                ? sensor_msgs::image_encodings::RGB8
                                : sensor_msgs::image_encodings::BGR8;
      cv_ptr = cv_bridge::toCvShare(img, enc);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat &mat = cv_ptr->image;
    if (mat.empty()) return;

    cv::Mat gray_full;
    if (mat.channels() == 1)
      gray_full = mat;
    else
    {
      const int code = (preferred_encoding_ == "rgb8") ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY;
      cv::cvtColor(mat, gray_full, code);
    }

    cv::Mat gray = gray_full;
    const double s = detect_scale_;
    if (s > 0.0 && s < 1.0)
      cv::resize(gray_full, gray, cv::Size(), s, s, cv::INTER_AREA);

    std::vector<cv::Point2f> corners;
    bool found = false;

    if (use_sb_)
    {
      found = cv::findChessboardCornersSB(gray, board_size_, corners);
    }
    else
    {
      int flags = 0;
      if (adaptive_thresh_) flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
      if (normalize_image_) flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
      if (fast_check_)      flags |= cv::CALIB_CB_FAST_CHECK;
      found = cv::findChessboardCorners(gray, board_size_, corners, flags);
    }

    if (!found) return;

    if (s > 0.0 && s < 1.0)
    {
      const double inv = 1.0 / s;
      for (auto &p : corners)
      {
        p.x = static_cast<float>(p.x * inv);
        p.y = static_cast<float>(p.y * inv);
      }
    }

    if (refine_corners_)
    {
      const int w = std::max(3, refine_win_);
      const cv::TermCriteria term(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
        std::max(1, refine_max_iters_), refine_eps_);
      cv::cornerSubPix(gray_full, corners, cv::Size(w, w), cv::Size(-1, -1), term);
    }

    cv::Mat rvec, tvec;
    const bool ok = cv::solvePnP(
      board_object_points_, corners, K_, D_, rvec, tvec,
      false, cv::SOLVEPNP_ITERATIVE);

    if (!ok) return;

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    tf2::Matrix3x3 m(
      Rcv.at<double>(0, 0), Rcv.at<double>(0, 1), Rcv.at<double>(0, 2),
      Rcv.at<double>(1, 0), Rcv.at<double>(1, 1), Rcv.at<double>(1, 2),
      Rcv.at<double>(2, 0), Rcv.at<double>(2, 1), Rcv.at<double>(2, 2));

    tf2::Quaternion q;
    m.getRotation(q);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = img->header.stamp;
    tf_msg.header.frame_id = img->header.frame_id.empty() ? "camera_frame" : img->header.frame_id;
    tf_msg.child_frame_id = "checkerboard";
    tf_msg.transform.translation.x = tvec.at<double>(0);
    tf_msg.transform.translation.y = tvec.at<double>(1);
    tf_msg.transform.translation.z = tvec.at<double>(2);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    cMo_->publish(tf_msg);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CheckerboardObjectNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
