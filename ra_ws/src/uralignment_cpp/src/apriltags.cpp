#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpPose.h>
#include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include <string>                    // String Operations
#include "rclcpp/rclcpp.hpp"         // ROS2 CPP API
#include "std_msgs/msg/string.hpp"   // ROS2 String Message
#include "sensor_msgs/msg/image.hpp" // ROS2 Image Message
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>
#include <Eigen/Geometry>
#include <visp3/vision/vpPose.h>
#include <ament_index_cpp/get_package_share_directory.hpp> // File Paths for .yamls
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/tf2/LinearMath/Quaternion.hpp>        // for matrix quaternion
#include <tf2/tf2/LinearMath/Matrix3x3.h>

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_UR_RTDE) && VISP_NAMESPACE_NAME
//! [Macro defined]
using namespace VISP_NAMESPACE_NAME;
#endif

class AprilTagNode : public rclcpp::Node
{
public:
  AprilTagNode() : Node("apriltag_node")
  {
    RCLCPP_INFO(this->get_logger(), "AprilTagNode initialized.");
    // Parameters
    width_ = declare_parameter<int>("width",1920);
    height_ = declare_parameter<int>("height",1080);
    fps_ = declare_parameter<int>("fps",30);  
    // Subscribe to Camera node's topic.
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&AprilTagNode::apriltag_loop, this, std::placeholders::_1));

    // Publish cMo:
    cMo_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("cMo", 10);
    
    initialization();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Declare shared pointer to subscription
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_; // Declare shared pointer to publication

  double tagSize = 0.0221;
  float quad_decimate = 1.0;
  int nThreads = 1;
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  cv::Mat latest_image_; // Hold latest image
  vpCameraParameters cam;
  vpTranslationVector t;   // (tx, ty, tz)
  vpRotationMatrix  R;     // 3x3 rotation
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  //! [Create AprilTag detector]
  vpDetectorAprilTag detector;
  //! [Create AprilTag detector]
  std::map<int, int> tags_index;
  std::string package_path = ament_index_cpp::get_package_share_directory("uralignment_cpp");
  // Params
  int width_{1920};
  int height_{1080};
  int fps_{30};

  void initialization()
  {
    // Camera Intrinsics:
    cam.initPersProjWithoutDistortion(  //diplays
      907.7258031, // px - Focal Length
      906.8582153, // py - Focal Length
      657.2998502, // u0 - Princicple Point
      358.9750977  // v0 - Principle Point
    );
    //! [AprilTag detector settings]
    detector = vpDetectorAprilTag(tagFamily);
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(align_frame);
    //! [AprilTag detector settings]

    tags_index.clear();
    tags_index[1] = 1; // blue up
    tags_index[2] = 2; // white left
    tags_index[3] = 3; // brown down
    tags_index[4] = 0; // black right


    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads: " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;
  }

  void apriltag_loop(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Define ViSP image buffers matching RealSense camera resolution:
    vpImage<vpRGBa> I_color(height_, width_); // Color
    vpImage<unsigned char> I(height_, width_); // Grayscale

    std::vector<vpColVector> trackedHoles(4);
    std::vector<bool> detectedMask(4, false);
    std::vector<vpColVector> filteredModelHoles, filteredTrackedHoles;

    // Model definition:
    std::vector<vpColVector> modelHoles(4);
    for (int i = 0; i < modelHoles.size(); ++i)
    {
      modelHoles[i].resize(3); // to avoid segmentation fault
    }
    modelHoles[0] = {0.06512, 0.0, 0.0}; // TODO parametrize this
    modelHoles[1] = {0.0, 0.06512, 0.0};
    modelHoles[2] = {-0.06512, 0.0, 0.0};
    modelHoles[3] = {0.0, -0.06512, 0.0};

    try
    {
      // Convert ROS2 image message to OpenCV-compatible cv::Mat:
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
      latest_image_ = cv_ptr->image.clone(); // Copy image to latest_image_

      if (latest_image_.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for camera image...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //continue;
      }

      // Convert OpenCV BGR image to vpImage<vpRGBa>:
      for (int i = 0; i < latest_image_.rows; ++i)
      {
        for (int j = 0; j < latest_image_.cols; ++j)
        {
          const cv::Vec3b &pixel = latest_image_.at<cv::Vec3b>(i, j);
          I_color[i][j] = vpRGBa(pixel[2], pixel[1], pixel[0]); // BGR to RGBa
        }
      }

      // Display RGBa & then Convert it to unsigned char vpImage, I, for AprilTag detection
      vpDisplay::display(I_color);
      vpImageConvert::convert(I_color, I);

      // Tag Detection:
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, tagSize, cam, cMo_vec);
      std::vector<int> tags_id = detector.getTagsId();

      filteredModelHoles.clear();
      filteredTrackedHoles.clear();
      for (int i = 0; i < trackedHoles.size(); ++i)
      {
        trackedHoles[i].resize(3); // to avoid segmentation fault
      }
      if (cMo_vec.size() >= 3 && cMo_vec.size() <= 4)
      {
        for (size_t i = 0; i < cMo_vec.size(); i++)
        {
          // vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3); // For each AprilTag
          vpTranslationVector t;
          cMo_vec[i].extract(t);

          int modelIndex = tags_index[tags_id[i]];
          trackedHoles[modelIndex] = {t[0], t[1], t[2]}; // routine
          detectedMask[modelIndex] = true;

          // Push the corresponding model and tracked holes into filtered vectors:
          filteredModelHoles.push_back(modelHoles[modelIndex]);
          filteredTrackedHoles.push_back(trackedHoles[modelIndex]);
        }
        // Passes the filtered vectors to findPose:
        vpHomogeneousMatrix cMo = findPose(filteredModelHoles, filteredTrackedHoles);
        cMo = findPose(filteredModelHoles, filteredTrackedHoles); // TODO write algorithm directly for column vectors
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

        // To save current cMo in .yaml:
        vpPoseVector cPo(cMo);
        std::stringstream ss_cPo;
        //std::cout << "cPo: " << cPo.t() << std::endl;
        ss_cPo << package_path + "/data/ur_pose_cPo.yaml";
        cPo.saveYAML(ss_cPo.str(), cPo); // Save target pose in camera frame


        cMo.extract(t);
        std::cout << "t:\n" << t << "\n";
        cMo.extract(R);
        std::cout << "R:\n" << R << "\n";
        tf2::Matrix3x3 m
          (
          R[0][0], R[0][1], R[0][2],
          R[1][0], R[1][1], R[1][2],
          R[2][0], R[2][1], R[2][2]
          );

        // Convert rotation matrix to quaternion
        tf2::Quaternion q;
        m.getRotation(q);
        std::cout << "q:\n" << q << "\n";

        // Build TransformStamped
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "camera_frame";
        tf_msg.child_frame_id  = "apriltag";
        tf_msg.transform.translation.x = t[0];
        tf_msg.transform.translation.y = t[1];
        tf_msg.transform.translation.z = t[2];
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        // Publish
        cMo_->publish(tf_msg);
        std::cout << "cMo_(Quaternion):\n" << cMo_ << "\n";
      }
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
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
  // VisP wrapper
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}