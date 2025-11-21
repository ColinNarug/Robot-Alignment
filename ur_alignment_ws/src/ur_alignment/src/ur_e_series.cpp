#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

// --- Global atomic flags ---
static std::atomic<char>  g_last_key{' '};
static std::atomic<bool>  g_send_vel{false};
static std::atomic<bool>  g_should_quit{false};

// --- Helper: load pose from YAML (supports both formats) ---
vpPoseVector loadPoseYaml(const std::string& path)
{
  YAML::Node config = YAML::LoadFile(path);
  vpPoseVector pose;

  if (config["data"]) {
    // Format: rows/cols/data
    for (size_t i = 0; i < 6; ++i) {
      pose[i] = config["data"][i][0].as<double>();
    }
  } else if (config["t"] && config["r"]) {
    // Format: t: [tx, ty, tz], r: [rx, ry, rz]
    auto t = config["t"].as<std::vector<double>>();
    auto r = config["r"].as<std::vector<double>>();
    for (int i = 0; i < 3; ++i) pose[i]   = t[i];
    for (int i = 0; i < 3; ++i) pose[i+3] = r[i];
  } else {
    throw std::runtime_error("Unsupported YAML format in " + path);
  }

  return pose;
}

class UniversalRobots_E_Series : public rclcpp::Node
{
public:
  UniversalRobots_E_Series() : Node("ur_node")
  {
    // --- Subscriptions ---
    cMo_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/cMo", 10, std::bind(&UniversalRobots_E_Series::run_servo_loop, this, std::placeholders::_1));

    rclcpp::QoS key_qos(1);
    key_qos.reliable().transient_local();
    key_sub_ = this->create_subscription<std_msgs::msg::Char>(
      "/teleop/last_key", key_qos, std::bind(&UniversalRobots_E_Series::key_callback, this, std::placeholders::_1));

    // --- Publishers ---
    v_c_pub_       = this->create_publisher<std_msgs::msg::Float64MultiArray>("v_c", 10);
    error_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("error", 10);
    errors_xyz_pub_= this->create_publisher<std_msgs::msg::Float64MultiArray>("errors_xyz", 10);

    initialization();
  }

private:
  // ROS handles
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_sub_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_pub_, error_pub_, errors_xyz_pub_;

  // Robot
  vpRobotUniversalRobots robot_;
  std::string robot_ip_ = "192.168.1.101";

  // Config
  std::string eMc_filename_, cdMo_filename_;
  vpHomogeneousMatrix eMc_, cdMo_;

  // Task
  vpServo task_;
  bool features_added_ = false;

  // Features
  vpFeatureTranslation t_, td_;
  vpFeatureThetaU tu_, tud_;
  vpColVector v_c_;

  // Initialization: connect robot + load config
  void initialization()
  {
    std::cout << "Attempt to establish connection..." << std::endl;
    robot_.connect(robot_ip_);
    std::cout << "WARNING: This program will move the robot! Please keep the stop button at hand!" << std::endl;

    std::string pkg_share = ament_index_cpp::get_package_share_directory("ur_alignment");
    eMc_filename_  = pkg_share + "/config/ur_eMc.yaml";
    cdMo_filename_ = pkg_share + "/config/ur_cdMo.yaml";

    // Load poses from YAML
    vpPoseVector ePc = loadPoseYaml(eMc_filename_);
    vpPoseVector cdPo= loadPoseYaml(cdMo_filename_);

    eMc_  = vpHomogeneousMatrix(ePc);
    cdMo_ = vpHomogeneousMatrix(cdPo);

    robot_.set_eMc(eMc_);

    // Servo task config
    task_.setLambda(0.1);
    task_.setServo(vpServo::EYEINHAND_CAMERA);
    task_.setInteractionMatrixType(vpServo::CURRENT);

    v_c_.resize(6);
    RCLCPP_INFO(this->get_logger(), "ur_e_series node started!");
  }

  // Key callback
  void key_callback(const std_msgs::msg::Char::SharedPtr msg)
  {
    char k = msg->data;
    g_last_key.store(k, std::memory_order_relaxed);

    if (k == 'x' || k == 'X') {
      g_send_vel.store(true,  std::memory_order_relaxed);
      g_should_quit.store(false,std::memory_order_relaxed);
      RCLCPP_INFO(this->get_logger(), "Key=X/x -> START");
    } else if (k == ' ') {
      g_send_vel.store(false, std::memory_order_relaxed);
      g_should_quit.store(false,std::memory_order_relaxed);
      RCLCPP_INFO(this->get_logger(), "Key=Space -> STOP");
    } else if (k == 'q' || k == 'Q' || k == 0x1B) {
      g_send_vel.store(false, std::memory_order_relaxed);
      g_should_quit.store(true, std::memory_order_relaxed);
      RCLCPP_WARN(this->get_logger(), "Key=Esc/Q/q -> QUIT");
    }
  }

  // Servo loop callback (semplificato)
  void run_servo_loop(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    // Convert ROS Transform -> ViSP
    vpTranslationVector c_t(msg->transform.translation.x,
                            msg->transform.translation.y,
                            msg->transform.translation.z);
    vpQuaternionVector c_q(msg->transform.rotation.x,
                           msg->transform.rotation.y,
                           msg->transform.rotation.z,
                           msg->transform.rotation.w);
    vpRotationMatrix c_R; c_R.buildFrom(c_q);

    std::cout << "t:\n" << c_t << "\nR:\n" << c_R << "\n";

    vpHomogeneousMatrix cMo(c_t, c_R);
    vpHomogeneousMatrix cdMc = cdMo_ * cMo.inverse();

    // Add features once
    if (!features_added_) {
      t_  = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tu_ = vpFeatureThetaU     (vpFeatureThetaU::cdRc);
      td_ = vpFeatureTranslation(vpFeatureTranslation::cdMc);
      tud_= vpFeatureThetaU     (vpFeatureThetaU::cdRc);
      task_.addFeature(t_, td_);
      task_.addFeature(tu_, tud_);
      features_added_ = true;
    }

    // Update features
    t_.buildFrom(cdMc);
    tu_.buildFrom(cdMc);

    // Compute control law
    if (g_should_quit.load()) {
      v_c_ = 0;
      robot_.setVelocity(vpRobot::CAMERA_FRAME, v_c_);
      rclcpp::shutdown();
      return;
    }
    if (g_send_vel.load()) {
      v_c_ = task_.computeControlLaw();
    } else {
      v_c_ = 0;
    }

    robot_.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    robot_.setVelocity(vpRobot::CAMERA_FRAME, v_c_);
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
