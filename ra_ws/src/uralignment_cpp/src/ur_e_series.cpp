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
#include <std_msgs/msg/char.hpp>
#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

std::atomic<char> last_key_{' '}; // starts “STOP”
std::atomic<bool> send_velocities_(false);
std::atomic<bool> should_quit_(false);

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
    cMo_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // cMo only
    rclcpp::SubscriptionOptions cMo_opts;
    cMo_opts.callback_group = cMo_handling_;
    sub_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // All other subscriptions
    rclcpp::SubscriptionOptions sub_opts;
    cMo_opts.callback_group = sub_handling_;
    compute_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // All UR_E_Series work and pubs
    rclcpp::SubscriptionOptions compute_opts;
    cMo_opts.callback_group = compute_handling_;
    
    // Subscriptions:
    cMo_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "cMo", 10, std::bind(&UniversalRobots_E_Series::cMo_callback, this, std::placeholders::_1), cMo_opts);
    rclcpp::QoS key_qos(1);
    key_qos.reliable().transient_local();  // get latched "last key" immediately
    key_sub_ = this->create_subscription<std_msgs::msg::Char>(
    "/teleop/last_key", key_qos, 
    std::bind(&UniversalRobots_E_Series::key_callback, this, std::placeholders::_1), sub_opts);
    offset_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/offset/cmd_twist_cf",
    rclcpp::SensorDataQoS(),
    std::bind(&UniversalRobots_E_Series::offset_twist_callback, this, std::placeholders::_1), sub_opts);

    // Publications:
    v_c_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("v_c", 10);
    error_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("error", 10);
    errors_xyz_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("errors_xyz", 10);
    pbvs_converged_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pbvs/converged", 10);

    // timer for compute_handling_ callback group
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
    std::bind(&UniversalRobots_E_Series::run_servo_loop, this), compute_handling_);

    initialization();
  }
  
private:
  rclcpp::CallbackGroup::SharedPtr cMo_handling_, sub_handling_, compute_handling_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_, error_, errors_xyz_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr offset_twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pbvs_converged_pub_;

  // PBVS Convergence Thresholds!
  double convergence_threshold_t = 0.001; // Value in [m]
  double convergence_threshold_tu = 0.1;  // Value in [deg]

  // Latch latest offset command
  geometry_msgs::msg::TwistStamped last_offset_twist_;
  rclcpp::Time last_offset_stamp_;

  vpRobotUniversalRobots robot; // Create an instance of vpRobotUniversalRobots Class named 'robot'
  std::string package_path = ament_index_cpp::get_package_share_directory("uralignment_cpp");
  std::string eMc_filename = package_path + "/config/ur_eMc.yaml";
  std::string cdMo_filename = package_path + "/config/ur_cdMo.yaml";
  std::string robot_ip = "192.168.1.101";
  bool opt_adaptive_gain; 
  bool opt_task_sequencing;
  bool final_quit;
  bool has_converged;
  bool servo_started;
  bool first_time; // Used for oMo Latching
  bool offset_twist_valid; // Latch offset velocity
  bool features_added; // For Error calc
  bool cMo_valid;
  bool is_quit_key(char k) const { return k == 'q' || k == 'Q' || k == 0x1B; }
  bool should_send_velocities(char k) const { return k == 'x' || k == 'X'; }
  vpPoseVector ePc, cdPo;
  vpHomogeneousMatrix cdMc, cdMo, oMo, eMc, cMo_data_;
  vpColVector v_c, errors_xyz, error; // velocity commands | Error in 6DOF | Trans/Rot Euclidian Norms
  std::vector<vpHomogeneousMatrix> v_oMo, v_cdMc;
  vpThetaUVector cd_tu_c;
  vpFeatureTranslation t, td;
  vpFeatureThetaU tu, tud;  
  vpTranslationVector cMo_t, cd_t_c;
  vpQuaternionVector cMo_q;
  vpRotationMatrix cMo_R;
  vpServo task;
  double error_tr;
  double error_tu;

  void initialization()
  {
    //! [Robot connection]
    std::cout << "Attempt to establish connection..." << std::endl;
    robot.connect(robot_ip);
    std::cout << "WARNING: This program will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl;
    //! [Robot connection]

    // Get camera extrinsics:
    ePc.loadYAML(eMc_filename, ePc);
    eMc.buildFrom(ePc);
    std::cout << "Read extrinsic parameters from file. eMc:\n" << eMc << "\n";
    // Build Desired Object relative to Camera Frame:
    cdPo.loadYAML(cdMo_filename, cdPo);
    cdMo.buildFrom(cdPo);
    std::cout << "Read Desired Transformation from file. cdMo:\n" << cdMo << "\n";
    robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    v_c.resize(6);
    errors_xyz.resize(6);
    error.resize(2);
    v_oMo.resize(6);
    v_cdMc.resize(6);
    cMo_valid =  false;
    cMo_t = vpTranslationVector(0.0, 0.0, 0.0);
    cMo_q = vpQuaternionVector(0.0, 0.0, 0.0, 1.0);
    cMo_R.buildFrom(cMo_q);
    cMo_data_.buildFrom(cMo_t, cMo_R);
    opt_adaptive_gain = false;
    opt_task_sequencing = false;
    final_quit = false;
    has_converged = false;
    offset_twist_valid = false;
    features_added = false;
    servo_started = false;
    first_time = true;

    if (opt_adaptive_gain) // Tuneable Gains
    {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else
    {
      task.setLambda(0.2); // Overall speed
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    RCLCPP_INFO(this->get_logger(), "ur_e_series node started!");
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

  void offset_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) // Closing Distance v_c
  {
    if (!msg) return;
    last_offset_twist_ = *msg;
    last_offset_stamp_ = this->now();
    offset_twist_valid = true;
  }

  void key_callback(const std_msgs::msg::Char::SharedPtr msg) // teleop_key callback
  {
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
  }

  void run_servo_loop()
  {
    if(!cMo_valid){return;}
    cdMc = cdMo * cMo_data_.inverse();

    if(first_time) // oMo cannot be found until receives correct cMo ( i.e. not in initialization() )
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
      first_time = false; // Do not return to cMo calc
      std::cout << "oMo:\n" << oMo << std::endl;
    }

    try
    {
      static double t_init_servo = vpTime::measureTimeMs(); // Initialized once
      const char k = last_key_.load(std::memory_order_relaxed);
      cdMc = cdMo * oMo * cMo_data_.inverse();
      t = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tu = vpFeatureThetaU(vpFeatureThetaU::cdRc);
      t.buildFrom(cdMc);
      tu.buildFrom(cdMc);
      cd_t_c = cdMc.getTranslationVector();
      cd_tu_c = cdMc.getThetaUVector();
      error_tr = sqrt(cd_t_c.sumSquare());
      error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare()));
      if (!has_converged && error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu)
      {
        const double max_age_s = 1.0; // 100ms safety watchdog
        const double age_s = (this->now() - last_offset_stamp_).seconds();
        has_converged = true;
        std_msgs::msg::Bool conv_msg;
        conv_msg.data = true;
        pbvs_converged_pub_->publish(conv_msg);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Offset twist stale/invalid -> zeroing v_c (valid=%d, age=%.3fs, max=%.3fs)",
          offset_twist_valid, age_s, max_age_s);
      }
      td = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tud = vpFeatureThetaU(vpFeatureThetaU::cdRc);
      if (!features_added)
      {
      task.addFeature(t, td);
      task.addFeature(tu, tud);
      features_added = true;
      }

      if (is_quit_key(k))
      {
        v_c = 0;
        try 
        {
          robot.setVelocity(has_converged ? vpRobot::END_EFFECTOR_FRAME : vpRobot::CAMERA_FRAME, v_c);
        } 
        catch (...) {}
        RCLCPP_WARN(this->get_logger(), "Quit key received. Stopping robot then shutting down...");
        rclcpp::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
        return;
      }

      const bool enabled = should_send_velocities(k);
      if (!enabled)
      {
        v_c = 0;
      }
      else if (!has_converged)
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
        const double max_age_s = 0.1;  // 100 ms watchdog
        const double age_s = (this->now() - last_offset_stamp_).seconds();
        if (offset_twist_valid && age_s < max_age_s)
        {
          v_c[0] = last_offset_twist_.twist.linear.x;
          v_c[1] = last_offset_twist_.twist.linear.y;
          v_c[2] = last_offset_twist_.twist.linear.z;
          v_c[3] = last_offset_twist_.twist.angular.x;
          v_c[4] = last_offset_twist_.twist.angular.y;
          v_c[5] = last_offset_twist_.twist.angular.z;
        }
        else
        {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Offset twist stale/invalid -> zeroing v_c (valid=%d, age=%.3fs, max=%.3fs)",
            offset_twist_valid, age_s, max_age_s);
          v_c = 0;
        }
      }
      robot.setVelocity(has_converged ? vpRobot::END_EFFECTOR_FRAME : vpRobot::CAMERA_FRAME, v_c);

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
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UniversalRobots_E_Series>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}