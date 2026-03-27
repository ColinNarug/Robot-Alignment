// This node is to use one Apriltag as the target object

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#endif
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpPlot.h>
#include <string>                    // String Operations
#include "rclcpp/rclcpp.hpp"         // ROS2 CPP API
#include "sensor_msgs/msg/image.hpp" // ROS2 Image Message
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp> // File Paths for .yamls
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <cstring>
#include <filesystem>
#include <algorithm>
#include <stdexcept>

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_UR_RTDE) && VISP_NAMESPACE_NAME
//! [Macro defined]
using namespace VISP_NAMESPACE_NAME;
#endif

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

class AprilTagNode : public rclcpp::Node
{
public:
  AprilTagNode() : Node("apriltag_node")
  {

    image_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cam_opts;
    cam_opts.callback_group = image_handling_;
    apriltag_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(this->get_logger(), "AprilTagNode initialized.");
    // Parameters
    width_ = declare_parameter<int>("width",1920);
    height_ = declare_parameter<int>("height",1080);
    fps_ = declare_parameter<int>("fps",30);
    active_camera_filename_ = this->declare_parameter<std::string>("active_camera_filename", "active_camera.yaml");
    // QoS controls
    qos_depth_ = this->declare_parameter<int>("qos_depth", 1);
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort"); // "best_effort" or "reliable"
    // Encoding
    preferred_encoding_ = this->declare_parameter<std::string>("preferred_encoding", "bgr8"); // "bgr8" or "rgb8"
    // AprilTag parameters
    nThreads = this->declare_parameter<int>("apriltag_threads", 2);
    tagSize = this->declare_parameter<double>("tag_size_m", 0.077);
    single_tag_id_ = this->declare_parameter<int>("single_tag_id", 1);
    // QoS
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
    qos.durability_volatile();
    if (qos_reliability_ == "reliable") qos.reliable();
    else qos.best_effort();
    // Subscribe to Camera node's topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", qos,
      std::bind(&AprilTagNode::image_callback, this, std::placeholders::_1), cam_opts);

    cMo_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("cMo", 10); // Publish cMo

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&AprilTagNode::apriltag_loop, this), apriltag_handling_);

    initialization();
  }

private:
  rclcpp::CallbackGroup::SharedPtr image_handling_, apriltag_handling_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Declare shared pointer to subscription
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_; // Declare shared pointer to publication
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::string active_camera_filename_{"active_camera.yaml"};
  ActiveCameraInfo active_;
  std::string active_camera_path_;
  std::string intrinsics_path_;
  bool have_cam_model_{false};

  // Params
  int width_{1920};
  int height_{1080};
  int fps_{30};
  int qos_depth_{1};
  std::string qos_reliability_{"best_effort"};
  std::string preferred_encoding_{"bgr8"};

  double tagSize = 0.077;
  int single_tag_id_{1};
  float quad_decimate = 1.0;
  int nThreads = 1;
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  vpCameraParameters cam;
  vpTranslationVector t;   // (tx, ty, tz)
  vpRotationMatrix R;     // 3x3 rotation
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  vpDetectorAprilTag detector;

  vpImage<unsigned char> I;
  std::vector<vpHomogeneousMatrix> cMo_vec;
  vpHomogeneousMatrix cMo;

  std::mutex img_mtx_;
  bool have_img_{false};

  void initialization()
  {
    I.resize(height_, width_); // Grayscale

    if (!load_active_camera_and_intrinsics_once())
    {
      throw std::runtime_error("Failed to load active camera / intrinsics YAML at startup.");
    }
    //! [AprilTag detector settings]
    detector = vpDetectorAprilTag(tagFamily);
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(align_frame);
    //! [AprilTag detector settings]

    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads: " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;
    std::cout << "single_tag_id: " << single_tag_id_ << std::endl;
    std::cout << "tag_size_m: " << tagSize << std::endl;
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

    const std::string intrinsics_fname =
      ac.serial + "_" + std::to_string(ac.width) + "x" + std::to_string(ac.height) + "_intrinsics.yaml";

    const std::string intr_path = find_first_existing(dirs, intrinsics_fname);
    if (intr_path.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not find matching intrinsics file: %s", intrinsics_fname.c_str());
      return false;
    }

    cv::Mat K, D;
    if (!read_intrinsics_yaml(intr_path, K, D))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read intrinsics YAML: %s", intr_path.c_str());
      return false;
    }

    cam.initPersProjWithoutDistortion(
      K.at<double>(0,0),
      K.at<double>(1,1),
      K.at<double>(0,2),
      K.at<double>(1,2)
    );

    active_ = ac;
    active_camera_path_ = ac_path;
    intrinsics_path_ = intr_path;
    have_cam_model_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Loaded active camera intrinsics: serial=%s %dx%d | %s",
      active_.serial.c_str(), active_.width, active_.height, intrinsics_path_.c_str());

    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg) return;
    std::scoped_lock lk(img_mtx_);
    latest_image_ = msg;
    have_img_ = true;
  }

  void apriltag_loop()
  {
    sensor_msgs::msg::Image::ConstSharedPtr img;
    {
      std::scoped_lock lk(img_mtx_);
      if (!have_img_ || !latest_image_) return;
      img = latest_image_;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      const std::string enc = (preferred_encoding_ == "rgb8") ? sensor_msgs::image_encodings::RGB8
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

    cv::Mat gray;
    if (mat.channels() == 1) gray = mat;
    else
    {
      const int code = (preferred_encoding_ == "rgb8") ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY;
      cv::cvtColor(mat, gray, code);
    }

    const int h = gray.rows, w = gray.cols;
    if ((int)I.getHeight() != h || (int)I.getWidth() != w)
    {
      I.resize(h, w);
    }

    for (int i = 0; i < h; ++i)
    {
      std::memcpy(I[i], gray.ptr<unsigned char>(i), static_cast<size_t>(w));
    }

    // Tag Detection:
    detector.detect(I, tagSize, cam, cMo_vec);
    const std::vector<int> tags_id = detector.getTagsId();

    if (tags_id.size() != cMo_vec.size())
    {
      return;
    }

    bool found_requested_tag = false;
    for (size_t i = 0; i < tags_id.size(); ++i)
    {
      if (tags_id[i] != single_tag_id_)
      {
        continue;
      }

      cMo = cMo_vec[i];
      found_requested_tag = true;
      break;
    }

    if (!found_requested_tag)
    {
      return;
    }

    cMo.extract(t);
    cMo.extract(R);
    tf2::Matrix3x3 m
    (
      R[0][0], R[0][1], R[0][2],
      R[1][0], R[1][1], R[1][2],
      R[2][0], R[2][1], R[2][2]
    );

    tf2::Quaternion q;
    m.getRotation(q);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = img->header.stamp;
    tf_msg.header.frame_id = "camera_frame";
    tf_msg.child_frame_id = "apriltag";
    tf_msg.transform.translation.x = t[0];
    tf_msg.transform.translation.y = t[1];
    tf_msg.transform.translation.z = t[2];
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    cMo_->publish(tf_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilTagNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 6);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}