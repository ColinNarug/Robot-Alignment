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
#include <std_msgs/msg/bool.hpp>
std::atomic<char> last_key_{' '}; // starts “STOP”
std::atomic<bool> send_velocities_(false);
std::atomic<bool> should_quit_(false);
#include <std_msgs/msg/char.hpp>
#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_UR_RTDE) && VISP_NAMESPACE_NAME
//! [Macro defined]
using namespace VISP_NAMESPACE_NAME;
#endif

class UniversalRobots_E_Series : public rclcpp::Node
{
public:
  UniversalRobots_E_Series() : Node("ur_node")
  {
    // Subscriptions:
    cMo_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "cMo", 10, std::bind(&UniversalRobots_E_Series::run_servo_loop, this, std::placeholders::_1));

    rclcpp::QoS key_qos(1);
    key_qos.reliable().transient_local();  // get latched "last key" immediately
    key_sub_ = this->create_subscription<std_msgs::msg::Char>(
    "/teleop/last_key", key_qos, std::bind(&UniversalRobots_E_Series::key_callback, this, std::placeholders::_1));

    // Publications:
    v_c_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("v_c", 10);
    error_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("error", 10);
    errors_xyz_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("errors_xyz", 10);

    initialization();

  }
private:
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_; // Declare shared pointer to subscription
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_, error_, errors_xyz_; // Declare shared pointer to publication
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_;

  // Create an instance of vpRobotUniversalRobots Class named 'robot'
  vpRobotUniversalRobots robot;
  std::string package_path = ament_index_cpp::get_package_share_directory("uralignment_cpp");
  std::string eMc_filename = package_path + "/config/ur_eMc.yaml";
  std::string cdMo_filename = package_path + "/config/ur_cdMo.yaml";
  std::string robot_ip = "192.168.1.101";
  double convergence_threshold_t = 0.00001; // Value in [m]
  double convergence_threshold_tu = 0.001;  // Value in [deg]
  vpColVector v_c; // This will store velocity commands
  vpColVector errors_xyz;
  vpColVector error;
  bool opt_adaptive_gain;
  bool opt_task_sequencing;
  bool final_quit;
  bool has_converged;
  bool servo_started;
  bool first_time;
  vpPoseVector ePc, cdPo;
  vpHomogeneousMatrix cdMc, cdMo, cMo, oMo, eMc, cMo_data_;
  vpServo task;
  std::vector<vpHomogeneousMatrix> v_oMo, v_cdMc;
  double error_tr;
  double error_tu;
  vpFeatureTranslation t;
  vpFeatureThetaU tu;  
  vpFeatureTranslation td;
  vpFeatureThetaU tud;

  bool is_quit_key(char k) const { return k == 'q' || k == 'Q' || k == 0x1B; }
  bool should_send_velocities(char k) const { return k == 'x' || k == 'X'; }

  void initialization()
  {
    //! [Robot connection]
    std::cout << "Attempt to establish connection..." << std::endl;
    robot.connect(robot_ip);
    std::cout << "WARNING: This program will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl;
    //! [Robot connection]

    v_c.resize(6);
    errors_xyz.resize(6);
    error.resize(2);
    v_oMo.resize(6);
    v_cdMc.resize(6);

    opt_adaptive_gain = false;
    opt_task_sequencing = false;
    final_quit = false;
    has_converged = false;
    //send_velocities = false;
    servo_started = false;
    first_time = true;

    // TODO tune this | TODO: CHECK THAT THIS ONLY NEEDS TO BE RUN ONCE!!!!
    if (opt_adaptive_gain) 
    {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else
    {
      task.setLambda(0.2); // TODO tune it
    }

    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    RCLCPP_INFO(this->get_logger(), "ur_e_series node started!");
  }


  void key_callback(const std_msgs::msg::Char::SharedPtr msg)
  {
    std::cout << "Top of key_callback" << std::endl;
    const char k = msg->data;
    last_key_.store(k, std::memory_order_relaxed);

    if (k == 'x' || k == 'X') 
    { 
      send_velocities_.store(true,  std::memory_order_relaxed);
      should_quit_.store(false,     std::memory_order_relaxed);
      RCLCPP_INFO(this->get_logger(), "Key=X/x -> START (send_velocities=true)");
    } 
    else if (k == ' ') 
    {
      send_velocities_.store(false, std::memory_order_relaxed);
      should_quit_.store(false,     std::memory_order_relaxed);
      RCLCPP_INFO(this->get_logger(), "Key=Space -> STOP (send_velocities=false)");
    } 
    else if (k == 'q' || k == 'Q' || k == 0x1B) 
    {
      send_velocities_.store(false, std::memory_order_relaxed);
      should_quit_.store(true,      std::memory_order_relaxed);
      RCLCPP_WARN(this->get_logger(), "Key=Esc/Q/q -> QUIT requested");
    } 
    else 
    {
      // Ignore any other key
    }
    std::cout << "Bottom of Key Callback" << std::endl;
  }


  void run_servo_loop(const geometry_msgs::msg::TransformStamped::SharedPtr msg) // cMo Callback
  {
    /*
    std::cout << "Top of run_servo_loop" << std::endl;
    for (int i = 0; i < 4; ++i) 
    {
      for (int j = 0; j < 4; ++j) 
      {
        cMo_data_[i][j] = msg->data[i * 4 + j];
      }
    }
    std::cout << "cMo:\n" << cMo_data_ << std::endl;
    */

    // --- Convert TransformStamped -> ViSP homogeneous matrix (cMo) ---
    const auto& T = msg->transform;

    // Translation
    vpTranslationVector cMo_t(T.translation.x, T.translation.y, T.translation.z);
    std::cout << "cMo_t:\n" << cMo_t << "\n";
    // Rotation: geometry_msgs quaternion -> ViSP quaternion -> rotation matrix
    vpQuaternionVector cMo_q(T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w);
    std::cout << "cMo_q:\n" << cMo_q << "\n";
    vpRotationMatrix cMo_R;
    cMo_R.buildFrom(cMo_q);
    std::cout << "cMo_R:\n" << cMo_R << "\n";

    // Rebuild the old working matrix container
    cMo_data_ = vpHomogeneousMatrix(cMo_t, cMo_R);
    std::cout << "cMo_data_:\n" << cMo_data_ << "\n";


    // Get camera extrinsics:
    ePc.loadYAML(eMc_filename, ePc);
    vpHomogeneousMatrix eMc(ePc);
    std::cout << "Read extrinsic parameters from file. eMc:\n" << eMc << "\n";

    // Build Desired Object relative to Camera Frame:
    cdPo.loadYAML(cdMo_filename, cdPo);
    vpHomogeneousMatrix cdMo(cdPo);
    std::cout << "Read Desired Transformation from file. cdMo:\n" << cdMo << "\n";

    robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    cdMc = cdMo * cMo_data_.inverse();
    std::cout << "cdMc:\n" << cdMc << std::endl;

    if (first_time) // Flip Logic:
    {
      RCLCPP_INFO(this->get_logger(), "first_time = true!");
      // Introduce security wrt tag positioning in order to avoid PI rotation (flip logic):
      v_oMo.resize(2);
      v_oMo[0].buildFrom(0, 0, 0, 0, 0, 0);
      v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
      std::cout << "First v_oMo[1]:\n" << v_oMo[1] << std::endl;
      for (size_t i = 0; i < 2; i++)
      {
        v_cdMc[i] = cdMo * v_oMo[i] * cMo_data_.inverse();
      }
      if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta()))
      {
        oMo = v_oMo[0];
      }
      else
      {
        std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
        oMo = v_oMo[1]; // Introduce PI rotation
      }
      first_time = false;
      std::cout << "oMo:\n" << oMo << std::endl;
    }

    try
    {
      static double t_init_servo = vpTime::measureTimeMs(); // TODO benchmarking time as in detection
      char k = last_key_.load(std::memory_order_relaxed);
      std::cout << "Last Key: " << last_key_ << std::endl;

      std::cout << "cdMo:\n" << cdMo << std::endl;
      cdMc = cdMo * oMo * cMo_data_.inverse();
      std::cout << "cdMc:\n" << cdMc << std::endl;
      t = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tu = vpFeatureThetaU(vpFeatureThetaU::cdRc);
      t.buildFrom(cdMc);
      tu.buildFrom(cdMc);
 
      vpTranslationVector cd_t_c = cdMc.getTranslationVector();
      vpThetaUVector cd_tu_c = cdMc.getThetaUVector();
      std::cout << "cd_t_c: \n" << cd_tu_c << std::endl;
      std::cout << "cd_tu_c: \n" << cd_tu_c << std::endl;
      td = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tud = vpFeatureThetaU(vpFeatureThetaU::cdRc);
      task.addFeature(t, td);
      task.addFeature(tu, tud);

      if (is_quit_key(k)) // Stop Robot & Quit Node
      {
        v_c = 0;
        try {robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);} 
        catch (...) {} // swallow errors to guarantee shutdown path
        RCLCPP_WARN(this->get_logger(), "Quit key received. Stopping robot then shutting down...");
        rclcpp::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
        return; // leave the callback
      }

      if (should_send_velocities(k)) // Start/Stop Robot
      {
        if (opt_task_sequencing) 
        {
          if (!servo_started) 
          {
            servo_started = true;
            t_init_servo = vpTime::measureTimeMs();
          }
          v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.0);
        }
        else 
        {
          v_c = task.computeControlLaw();
        }
      } 
      else 
      {
        v_c = 0;
      }

      //errors_xyz = task.getError();
      std::cout << "errors_xyz: \n" << errors_xyz << std::endl;
      error_tr = sqrt(cd_t_c.sumSquare());
      error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare()));
      std::cout << "error_tr: \n" << error_tr << std::endl;
      std::cout << "error_tu: \n" << error_tu << std::endl;

      if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu)
      {
        has_converged = true;
        std::cout << "Servo task has converged" << std::endl;
      }
      if (has_converged)
      {
        v_c = 0;
        RCLCPP_INFO(this->get_logger(), "Servo stopped: Target pose reached.");
      }

      robot.setVelocity(vpRobot::CAMERA_FRAME, v_c); // Send to the robot
      std::cout << "v_c: " << v_c.t() << std::endl;

      std_msgs::msg::Float64MultiArray vel;
      vel.layout.dim.resize(2);
      vel.layout.dim[0].label = "rows";
      vel.layout.dim[0].size = 6;
      vel.layout.dim[0].stride = 6;
      vel.layout.dim[1].label = "cols";
      vel.layout.dim[1].size = 1;
      vel.layout.dim[1].stride = 1;
      vel.data.resize(6);
      for (size_t i = 0; i < 6; ++i)
      {
        vel.data[i] = v_c[i];
      }
      v_c_->publish(vel);
      std::cout << "vel: " << v_c.t() << std::endl;


      std_msgs::msg::Float64MultiArray errs;
      errs.layout.dim.resize(2);
      errs.layout.dim[0].label = "rows";
      errs.layout.dim[0].size = 6;
      errs.layout.dim[0].stride = 6;
      errs.layout.dim[1].label = "cols";
      errs.layout.dim[1].size = 1;
      errs.layout.dim[1].stride = 1;
      errs.data.resize(6);
      for (int i = 0; i < 3; ++i) 
      {
        errors_xyz[i]   = t[i];
        errors_xyz[i+3] = tu[i];
      }
      for (size_t i = 0; i < 6; ++i)
      {
        errs.data[i] = errors_xyz[i];
      }
      errors_xyz_->publish(errs);
      std::cout << "errs: " << errors_xyz.t() << std::endl;

      std_msgs::msg::Float64MultiArray err;
      err.layout.dim.resize(2);
      err.layout.dim[0].label = "rows";
      err.layout.dim[0].size = 2;
      err.layout.dim[0].stride = 2;
      err.layout.dim[1].label = "cols";
      err.layout.dim[1].size = 1;
      err.layout.dim[1].stride = 1;
      err.data.resize(2);
      err.data[0] = error_tr;
      err.data[1] = error_tu;
      error_->publish(err);
      std::cout << "err: " << error.t() << std::endl;

    }
    catch (const vpException &e)
    {
      std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }

    catch (const std::exception &e)
    {
      std::cout << "ur_rtde exception: " << e.what() << std::endl;
      std::exit(EXIT_FAILURE);
    }
    std::cout << "Bottom of run_servo_loop" << std::endl;
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UniversalRobots_E_Series>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}