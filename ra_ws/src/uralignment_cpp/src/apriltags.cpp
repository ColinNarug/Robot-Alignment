#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#endif
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpPlot.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
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

struct ActiveCameraInfo // Declare for grouping related data
{
  std::string serial;
  int width{0};
  int height{0};
  bool valid() const { return !serial.empty() && width > 0 && height > 0; }
};

// Derive the workspace root by trimming the install pack off a package share path
static std::string infer_workspace_root_from_share(const std::string& share_dir)
{
  const std::string needle = "/install/";
  const auto pos = share_dir.find(needle);
  if (pos == std::string::npos) return "";
  return share_dir.substr(0, pos);
}

// Construct candidate configuraton directories across install and source mirrors
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

// Return the first directory entry containing the requested file name
static std::string find_first_existing(const std::vector<std::string>& dirs, const std::string& filename)
{
  for (const auto& d : dirs)
  {
    const std::string p = d + "/" + filename;
    if (fs::exists(p)) return p;
  }
  return "";
}

// Read YAML (active_camera.yaml) for serial and resolution
static bool read_active_camera_yaml(const std::string& path, ActiveCameraInfo& out)
{
  cv::FileStorage fsr(path, cv::FileStorage::READ);
  if (!fsr.isOpened()) return false;

  fsr["serial"] >> out.serial;
  fsr["width"]  >> out.width;
  fsr["height"] >> out.height;
  return out.valid();
}

// Read camera instrinsics and distortion coefficients from YAML
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
    // Callback Groups:
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
    // AprilTag threads
    nThreads = this->declare_parameter<int>("apriltag_threads", 2);
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
  
  double tagSize = 0.0221; // Black Square of the AprilTags
  float quad_decimate = 1.0;
  int nThreads = 1;
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  vpCameraParameters cam;
  vpTranslationVector t; // (tx, ty, tz)
  vpRotationMatrix  R; // 3x3 rotation
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  //! [Create AprilTag detector]
  vpDetectorAprilTag detector;
  //! [Create AprilTag detector]
  std::map<int, int> tags_index;
  std::string package_path = ament_index_cpp::get_package_share_directory("uralignment_cpp");

  vpImage<unsigned char> I; // Define ViSP image buffers matching RealSense camera resolution
  std::vector<vpColVector> trackedHoles;
  std::vector<bool> detectedMask;
  std::vector<vpColVector> filteredModelHoles, filteredTrackedHoles;
  // Model definition:
  std::vector<vpColVector> modelHoles;
  std::vector<vpHomogeneousMatrix> cMo_vec;
  vpHomogeneousMatrix cMo;
  std::mutex img_mtx_;
  bool have_img_{false};

  void initialization()
  {
    I.resize(height_, width_);

    trackedHoles.resize(4);
    detectedMask.resize(4, false);
        for (int i = 0; i < trackedHoles.size(); ++i)
    {
      trackedHoles[i].resize(3); // to avoid segmentation fault
    }
    
    modelHoles.resize(4);
    for (int i = 0; i < modelHoles.size(); ++i)
    {
      modelHoles[i].resize(3); // to avoid segmentation fault
    }
    modelHoles[0] = {0.06512, 0.0, 0.0}; // TODO parametrize this
    modelHoles[1] = {0.0, 0.06512, 0.0};
    modelHoles[2] = {-0.06512, 0.0, 0.0};
    modelHoles[3] = {0.0, -0.06512, 0.0};

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

    tags_index.clear();
    tags_index[1] = 1;
    tags_index[2] = 2;
    tags_index[3] = 3;
    tags_index[4] = 0; 

    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads: " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;
  }

  bool load_active_camera_and_intrinsics_once() // Load active camera intrinsics once at startup
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

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) // Save msg as latest_image_
  {
    if (!msg) return;
    std::scoped_lock lk(img_mtx_);
    latest_image_ = msg;
    have_img_ = true;
  }

  void apriltag_loop() // Find AprilTags concentric center and publish to topic
  {
      sensor_msgs::msg::Image::ConstSharedPtr img;
      {
        std::scoped_lock lk(img_mtx_); // Protect shared data
        if (!have_img_ || !latest_image_) return;
        img = latest_image_;
      }

      // Convertion (GUI thread)
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

      cv::Mat gray; // Declare to store grayscale image
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
      std::vector<int> tags_id = detector.getTagsId();

      filteredModelHoles.clear();
      filteredTrackedHoles.clear();
      std::fill(detectedMask.begin(), detectedMask.end(), false);

      if (cMo_vec.size() >= 3 && cMo_vec.size() <= 4 && tags_id.size() == cMo_vec.size())
      {
        for (size_t i = 0; i < cMo_vec.size(); i++)
        {
          vpTranslationVector t;
          cMo_vec[i].extract(t);

          auto it = tags_index.find(tags_id[i]);
          if (it == tags_index.end())
          {
            continue; // ignore unknown tag IDs
          }

          int modelIndex = it->second;
          trackedHoles[modelIndex] = {t[0], t[1], t[2]};
          detectedMask[modelIndex] = true;

          filteredModelHoles.push_back(modelHoles[modelIndex]);
          filteredTrackedHoles.push_back(trackedHoles[modelIndex]);
        }

        if (filteredModelHoles.size() < 3)
        {
          return;
        }

        // Passes the filtered vectors to findPose:
        cMo = findPose(filteredModelHoles, filteredTrackedHoles);
        if (cMo == vpHomogeneousMatrix())
        {
          std::cerr << "WARNING: cMo is identity. Pose estimation failed or returned invalid transform." << std::endl;
          // Print input data for debugging:
          std::cerr << "Filtered Model Holes:" << std::endl;
          for (const auto &pt : filteredModelHoles)
            std::cerr << pt.t() << std::endl;
          std::cerr << "Filtered Tracked Holes:" << std::endl;
          for (const auto &pt : filteredTrackedHoles)
            std::cerr << pt.t() << std::endl;
        }

        cMo.extract(t);
        cMo.extract(R);
        tf2::Matrix3x3 m
          (
          R[0][0], R[0][1], R[0][2],
          R[1][0], R[1][1], R[1][2],
          R[2][0], R[2][1], R[2][2]
          );

        // Convert rotation matrix to quaternion
        tf2::Quaternion q;
        m.getRotation(q);

        // Build TransformStamped
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
    
  }
  
  // Kabsch.cpp Start: ======================================================================
  Eigen::Affine3d Find3DAffineTransformSameScale(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out)
{
  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++)
  {
    in_ctr += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++)
  {
    in.col(col) -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD (single-value decomposition)
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = R;
  A.translation() = (out_ctr - R * in_ctr);

  return A;
}
  // ViSP wrapper
  vpHomogeneousMatrix findPose(std::vector<vpColVector> in, std::vector<vpColVector> out)
{
  // Create an Eigen Matrix to hold the points
  Eigen::Matrix3Xd inputPoints(3, in.size()), outputPoints(3, out.size());

  // Fill the Eigen matrix with the coordinates from vpColVector
  for (size_t i = 0; i < in.size(); ++i)
  {
    inputPoints(0, i) = in[i][0];
    inputPoints(1, i) = in[i][1];
    inputPoints(2, i) = in[i][2];
  }
  for (size_t i = 0; i < out.size(); ++i)
  {
    outputPoints(0, i) = out[i][0];
    outputPoints(1, i) = out[i][1];
    outputPoints(2, i) = out[i][2];
  }

  Eigen::Affine3d A = Find3DAffineTransformSameScale(inputPoints, outputPoints);

  vpHomogeneousMatrix cMo;
  cMo[0][0] = A.linear()(0, 0);
  cMo[0][1] = A.linear()(0, 1);
  cMo[0][2] = A.linear()(0, 2);
  cMo[0][3] = A.translation()(0);
  cMo[1][0] = A.linear()(1, 0);
  cMo[1][1] = A.linear()(1, 1);
  cMo[1][2] = A.linear()(1, 2);
  cMo[1][3] = A.translation()(1);
  cMo[2][0] = A.linear()(2, 0);
  cMo[2][1] = A.linear()(2, 1);
  cMo[2][2] = A.linear()(2, 2);
  cMo[2][3] = A.translation()(2);
  return cMo;
}
  // Kabsch.cpp End: ========================================================================

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