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
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>


#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party(X11, GDI or openCV) to dislay images..." << std::endl;
#else
  bool display_off = false;
#endif

class DisplayNode : public rclcpp::Node
{
public:
  DisplayNode() : Node("displays_node")
  {

    image_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cam_opts;
    cam_opts.callback_group = image_handling_;
    vector_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions vector_opts;
    cam_opts.callback_group = vector_handling_;

    // Parameters
    width_ = declare_parameter<int>("width",1920);
    height_ = declare_parameter<int>("height",1080);
    fps_ = declare_parameter<int>("fps",30);
    // QoS controls 
    qos_depth_ = this->declare_parameter<int>("qos_depth", 5);
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort"); // "best_effort" or "reliable"
    // Encoding
    preferred_encoding_ = this->declare_parameter<std::string>("preferred_encoding", "bgr8");  // "bgr8" or "rgb8"
    // QoS
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
    qos.durability_volatile();
    if (qos_reliability_ == "reliable") qos.reliable();
    else qos.best_effort();
    // Subscribe to Camera node's topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", qos,
    std::bind(&DisplayNode::image_callback, this, std::placeholders::_1), cam_opts);

    cMo_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "cMo", 10, std::bind(&DisplayNode::cMo_callback, this, std::placeholders::_1), vector_opts);

    v_c_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "v_c", 10, std::bind(&DisplayNode::v_c_callback, this, std::placeholders::_1), vector_opts);

    errors_xyz_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "errors_xyz", 10, std::bind(&DisplayNode::errors_xyz_callback, this, std::placeholders::_1), vector_opts);

    error_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "error", 10, std::bind(&DisplayNode::error_callback, this, std::placeholders::_1), vector_opts);

    initialization();
  }

private:
  rclcpp::CallbackGroup::SharedPtr image_handling_, vector_handling_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_, errors_xyz_, error_;  
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Params
  int width_{1920};
  int height_{1080};
  int fps_{30};
  int qos_depth_{5};
  std::string qos_reliability_{"best_effort"};
  std::string preferred_encoding_{"bgr8"};

  std::string package_path = ament_index_cpp::get_package_share_directory("uralignment_cpp");
  std::string cdMo_filename = package_path + "/config/ur_cdMo.yaml";
  bool opt_plot;
  bool display_initialized_;
  bool cMo_valid;
  vpColVector v_c_data_, error_data_, errors_xyz_data_;
  vpPoseVector cdPo;
  vpHomogeneousMatrix cdMo, cdMc, oMo, cMo_data_;
  vpFeatureTranslation t, td;
  vpFeatureThetaU tu, tud; 
  vpTranslationVector cMo_t, cd_t_c;
  vpQuaternionVector cMo_q;
  vpRotationMatrix cMo_R;
  cv::Mat latest_image_; // Hold latest image
  // Define ViSP image buffers matching RealSense camera resolution:
  vpImage<vpRGBa> I_color;
  vpImage<unsigned char> I;
  vpPlot *plotter = nullptr;
  int iter_plot;
  vpCameraParameters cam;
  vpServo task;
  std::stringstream ss;
  std::vector<vpImagePoint> *traj_vip = nullptr; // To memorize point trajectory
  double t_start;
  std::unique_ptr<vpDisplay> disp_;

  void initialization()
  {
    // Camera Intrinsics:
    cam.initPersProjWithoutDistortion(
      907.7258031,  // px - Focal Length
      906.8582153,  // py - Focal Length
      657.2998502,  // u0 - Princicple Point
      358.9750977); // v0 - Principle Point

    // Build Desired Object relative to Camera Frame:
    cdPo.loadYAML(cdMo_filename, cdPo);
    cdMo.buildFrom(cdPo);
    std::cout << "Read Desired Transformation from file. cdMo:\n" << cdMo << "\n";

    iter_plot = 0;
    I_color.resize(height_, width_);
    I.resize(height_, width_);
    plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
    opt_plot = true;
    t_start = vpTime::measureTimeMs();

    if(opt_plot)
    {
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      plotter->initGraph(0, 6);
      plotter->initGraph(1, 6);
      plotter->setLegend(0, 0, "error_feat_tx");
      plotter->setLegend(0, 1, "error_feat_ty");
      plotter->setLegend(0, 2, "error_feat_tz");
      plotter->setLegend(0, 3, "error_feat_theta_ux");
      plotter->setLegend(0, 4, "error_feat_theta_uy");
      plotter->setLegend(0, 5, "error_feat_theta_uz");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
    }

    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    cMo_valid = false;
    display_initialized_ = false;

    cMo_t = vpTranslationVector(0.0, 0.0, 0.0);
    cMo_q = vpQuaternionVector(0.0, 0.0, 0.0, 1.0);
    cMo_R.buildFrom(cMo_q);
    cMo_data_.buildFrom(cMo_t, cMo_R);

    std::cout << "Displays Node Started!" << std::endl;
  }

  void cMo_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) // cMo Callback
  {
    if(!msg) return;
    const auto& T = msg->transform;
    cMo_t[0] = T.translation.x;
    cMo_t[1] = T.translation.y;
    cMo_t[2] = T.translation.z;
    cMo_q[0] = T.rotation.x;
    cMo_q[1] = T.rotation.y;
    cMo_q[2] = T.rotation.z;
    cMo_q[3] = T.rotation.w;
    cMo_R.buildFrom(cMo_q);
    cMo_data_.buildFrom(cMo_t, cMo_R);
    cMo_valid = true;
  }

  void v_c_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() != 6)
      return;
    v_c_data_.resize(6);
    for (int i = 0; i < 6; ++i)
    {
      v_c_data_[i] = msg->data[i];
    }
  }

  void errors_xyz_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() != 6)
      return;
    errors_xyz_data_.resize(6);
    for (int i = 0; i < 6; ++i)
    {
      errors_xyz_data_[i] = msg->data[i];
    }
  }

  void error_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() != 2)
      return;
    error_data_.resize(2);
    for (int i = 0; i < 2; ++i)
    {
      error_data_[i] = msg->data[i];
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) // Runs when new image message received from subscription
  {
    cdMc = cdMo * cMo_data_.inverse();

    if (!display_initialized_) 
    {
#ifdef VISP_HAVE_X11
      disp_.reset(); // close previous (if any)
      disp_ = std::make_unique<vpDisplayX>(I_color, 100, 30, "Pose from Homography");
      display_initialized_ = true;
#else
      return; // no GUI backend available
#endif
    }

    try
    {
      t = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tu = vpFeatureThetaU(vpFeatureThetaU::cdRc);
      t.buildFrom(cdMc);
      tu.buildFrom(cdMc);
      td = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tud = vpFeatureThetaU(vpFeatureThetaU::cdRc);
      task.addFeature(t, td);
      task.addFeature(tu, tud);

      RCLCPP_INFO(this->get_logger(), "Image callback triggered! Image resolution: %d x %d", msg->width, msg->height);

      // Convert ROS2 image message to OpenCV-compatible cv::Mat:
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
      latest_image_ = cv_ptr->image.clone(); // Copy image to latest_image_
      // Convert OpenCV BGR image to vpImage<vpRGBa>:
      for (int i = 0; i < latest_image_.rows; ++i)
      {
        for (int j = 0; j < latest_image_.cols; ++j)
        {
          const cv::Vec3b &pixel = latest_image_.at<cv::Vec3b>(i, j);
          I_color[i][j] = vpRGBa(pixel[2], pixel[1], pixel[0]); // BGR to RGBa
        }
      }

      // Update visual features:
      cdMc = cdMo * oMo * cMo_data_.inverse();
      t.buildFrom(cdMc);
      tu.buildFrom(cdMc);

      if (disp_)
      {
        vpDisplay::display(I_color);
        vpImageConvert::convert(I_color, I);
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
        // Display desired and current pose features:
        vpDisplay::displayFrame(I_color, cdMo * oMo, cam, 0.06512, vpColor::yellow, 2);
        vpDisplay::displayFrame(I_color, cMo_data_, cam, 0.06512, vpColor::none, 3);
        // TODO display trajectory of Pose
        vpDisplay::flush(I_color);
      }

      if (opt_plot)
      {
      plotter->plot(0, iter_plot, errors_xyz_data_);
      plotter->plot(1, iter_plot, v_c_data_);
      iter_plot++;
      }

      ss.str("");
      ss << "error_t: " << error_data_[0];
      vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
      ss.str("");
      ss << "error_tu: " << error_data_[1];
      vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
      ss.str("");
      ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";

      vpDisplay::displayText(I_color, 40, 20, ss.str(), vpColor::red);
      vpDisplay::flush(I_color);
    }
    catch (const vpException &e)
    {
      std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}