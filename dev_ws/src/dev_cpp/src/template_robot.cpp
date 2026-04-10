#include <rclcpp/rclcpp.hpp> // Include the ROS node api
#include <std_msgs/msg/float64_multi_array.hpp> // Include the numeric array message type
#include <std_msgs/msg/bool.hpp> // Include the bool message type
#include <std_msgs/msg/char.hpp> // Include the char message type
#include <geometry_msgs/msg/transform_stamped.hpp> // Include the transform message type
#include <geometry_msgs/msg/twist_stamped.hpp> // Include the twist message type

#include <algorithm> // Include clamp helper functions
#include <atomic> // Include atomic shared state support
#include <chrono> // Include timer duration support
#include <cmath> // Include square root and acos support
#include <functional> // Include function binding support
#include <string> // Include string support

std::atomic<char> last_key_{' '}; // Store the latest teleop key value
std::atomic<bool> send_velocities_{false}; // Store the teleop motion latch
std::atomic<bool> should_quit_{false}; // Store the teleop quit latch

class TemplateRobotNode : public rclcpp::Node { // Define the template robot node
public:
  TemplateRobotNode() : Node("template_robot_node") { // Initialize the node with the original template name
    desired_offset_d_m_ = this->declare_parameter<double>("desired_offset_d_m", 0.2159); // Store the desired offset value
    robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.1.101"); // Store the robot ip string
    cMo_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the pose callback group
    rclcpp::SubscriptionOptions cMo_opts; // Create the pose subscription options
    cMo_opts.callback_group = cMo_handling_; // Assign the pose callback group
    sub_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Create the general callback group
    rclcpp::SubscriptionOptions sub_opts; // Create the general subscription options
    sub_opts.callback_group = sub_handling_; // Assign the general callback group
    compute_handling_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the compute callback group
    cMo_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "cMo",
      10,
      std::bind(&TemplateRobotNode::cMo_callback, this, std::placeholders::_1),
      cMo_opts); // Subscribe to the object pose topic
    rclcpp::QoS key_qos(1); // Build the latched teleop qos profile
    key_qos.reliable(); // Request reliable teleop delivery
    key_qos.transient_local(); // Request latched teleop delivery
    key_sub_ = this->create_subscription<std_msgs::msg::Char>(
      "/teleop/last_key",
      key_qos,
      std::bind(&TemplateRobotNode::key_callback, this, std::placeholders::_1),
      sub_opts); // Subscribe to the teleop topic
    offset_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/offset/cmd_twist",
      rclcpp::SensorDataQoS(),
      std::bind(&TemplateRobotNode::offset_twist_callback, this, std::placeholders::_1),
      sub_opts); // Subscribe to the offset twist topic
    v_c_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("v_c", 10); // Create the velocity publisher
    error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("error", 10); // Create the error norm publisher
    errors_xyz_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("errors_xyz", 10); // Create the component error publisher
    pbvs_converged_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pbvs/converged", 10); // Create the convergence publisher
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&TemplateRobotNode::run_servo_loop, this),
      compute_handling_); // Create the servo timer
    last_offset_stamp_ = this->now(); // Initialize the offset timestamp
    last_key_stamp_ = this->now(); // Initialize the key timestamp
    RCLCPP_INFO(
      this->get_logger(),
      "template robot node started desired offset=%f robot ip=%s",
      desired_offset_d_m_,
      robot_ip_.c_str()); // Report the template startup state
  }

private:
  rclcpp::CallbackGroup::SharedPtr cMo_handling_; // Store the pose callback group
  rclcpp::CallbackGroup::SharedPtr sub_handling_; // Store the general callback group
  rclcpp::CallbackGroup::SharedPtr compute_handling_; // Store the compute callback group
  rclcpp::TimerBase::SharedPtr timer_; // Store the periodic timer handle
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_sub_; // Store the pose subscription handle
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_; // Store the teleop subscription handle
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr offset_twist_sub_; // Store the offset subscription handle
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_pub_; // Store the velocity publisher handle
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_; // Store the error norm publisher handle
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr errors_xyz_pub_; // Store the component error publisher handle
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pbvs_converged_pub_; // Store the convergence publisher handle
  double desired_offset_d_m_{0.2159}; // Store the desired offset parameter
  std::string robot_ip_{"192.168.1.101"}; // Store the robot ip parameter
  geometry_msgs::msg::TwistStamped last_offset_twist_; // Store the latest offset twist message
  rclcpp::Time last_offset_stamp_{0, 0, RCL_ROS_TIME}; // Store the latest offset timestamp
  rclcpp::Time last_key_stamp_{0, 0, RCL_ROS_TIME}; // Store the latest key timestamp
  geometry_msgs::msg::TransformStamped last_cMo_; // Store the latest object transform message
  bool cMo_valid_{false}; // Track whether object data exists
  bool offset_twist_valid_{false}; // Track whether offset twist data exists
  bool has_converged_{false}; // Track the convergence latch state

  bool is_quit_key(char key) const { // Test whether the input key requests shutdown
    return key == 'q' || key == 'Q' || key == 0x1B; // Match the supported quit keys
  }

  bool should_send_velocities(char key) const { // Test whether the input key requests motion
    return key == 'x' || key == 'X'; // Match the supported start keys
  }

  void cMo_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) { // Save the latest object pose
    if (!msg) { // Ignore null transform pointers
      return; // Leave the callback when the message is empty
    }
    last_cMo_ = *msg; // Copy the latest object transform
    cMo_valid_ = true; // Mark the object pose as valid
  }

  void offset_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { // Save the latest offset twist
    if (!msg) { // Ignore null twist pointers
      return; // Leave the callback when the message is empty
    }
    last_offset_twist_ = *msg; // Copy the latest offset twist message
    last_offset_stamp_ = this->now(); // Update the offset timestamp
    offset_twist_valid_ = true; // Mark the offset command as valid
  }

  void key_callback(const std_msgs::msg::Char::SharedPtr msg) { // Process the teleop key input
    if (!msg) { // Ignore null key pointers
      return; // Leave the callback when the message is empty
    }
    const char key = msg->data; // Copy the received key value
    last_key_.store(key, std::memory_order_relaxed); // Save the latest key value atomically
    last_key_stamp_ = this->now(); // Update the key timestamp
    if (key == 'x' || key == 'X') { // Handle the start command
      send_velocities_.store(true, std::memory_order_relaxed); // Set the motion latch
      should_quit_.store(false, std::memory_order_relaxed); // Clear the quit latch
      RCLCPP_INFO(this->get_logger(), "template robot start command received"); // Report the start event
    } else if (key == ' ') { // Handle the stop command
      send_velocities_.store(false, std::memory_order_relaxed); // Clear the motion latch
      should_quit_.store(false, std::memory_order_relaxed); // Clear the quit latch
      RCLCPP_INFO(this->get_logger(), "template robot stop command received"); // Report the stop event
    } else if (is_quit_key(key)) { // Handle the quit command
      send_velocities_.store(false, std::memory_order_relaxed); // Clear the motion latch
      should_quit_.store(true, std::memory_order_relaxed); // Set the quit latch
      RCLCPP_WARN(this->get_logger(), "template robot quit command received"); // Report the quit event
    }
  }

  double quaternion_angle_deg() const { // Convert the stored quaternion into a rotation angle
    const double w = std::max(
      -1.0,
      std::min(1.0, static_cast<double>(last_cMo_.transform.rotation.w))); // Clamp the scalar term to a valid range
    const double angle_rad = 2.0 * std::acos(w); // Compute the equivalent angle in radians
    return angle_rad * 180.0 / 3.14159265358979323846; // Convert the angle into degrees
  }

  void publish_zero_velocity() { // Publish a zero velocity command for template safety
    std_msgs::msg::Float64MultiArray vel; // Create the velocity message
    vel.data.resize(6, 0.0); // Fill the velocity vector with zeros
    v_c_pub_->publish(vel); // Publish the zero velocity message
  }

  void publish_errors(double error_tr, double error_rot) { // Publish the template error messages
    std_msgs::msg::Float64MultiArray errs_xyz; // Create the component error message
    errs_xyz.data.resize(6, 0.0); // Allocate the component error vector
    errs_xyz.data[0] = last_cMo_.transform.translation.x; // Store the x translation error component
    errs_xyz.data[1] = last_cMo_.transform.translation.y; // Store the y translation error component
    errs_xyz.data[2] = last_cMo_.transform.translation.z; // Store the z translation error component
    errors_xyz_pub_->publish(errs_xyz); // Publish the component error message
    std_msgs::msg::Float64MultiArray err; // Create the error norm message
    err.data.resize(2, 0.0); // Allocate the error norm vector
    err.data[0] = error_tr; // Store the translation error norm
    err.data[1] = error_rot; // Store the rotation error norm
    error_pub_->publish(err); // Publish the error norm message
  }

  void publish_convergence(bool state) { // Publish the template convergence state
    std_msgs::msg::Bool msg; // Create the convergence message
    msg.data = state; // Store the convergence value
    pbvs_converged_pub_->publish(msg); // Publish the convergence message
  }

  void run_servo_loop() { // Run the safe template robot loop
    if (should_quit_.load(std::memory_order_relaxed)) { // Stop the process when a quit command exists
      publish_zero_velocity(); // Publish a zero velocity command before shutdown
      rclcpp::shutdown(); // Request ROS shutdown
      return; // Leave the timer callback after shutdown
    }
    if (!cMo_valid_) { // Wait until object data exists before computing errors
      publish_zero_velocity(); // Publish a zero velocity command while waiting
      return; // Leave the timer callback until pose data arrives
    }












  ///////////////////////////////////////////////////////
  /*
  Main development zone.
  Use object to calculate velocities.
  */
  ///////////////////////////////////////////////////////















    const double error_tr = std::sqrt(
      last_cMo_.transform.translation.x * last_cMo_.transform.translation.x +
      last_cMo_.transform.translation.y * last_cMo_.transform.translation.y +
      last_cMo_.transform.translation.z * last_cMo_.transform.translation.z); // Compute the translation error norm
    const double error_rot = quaternion_angle_deg(); // Compute the rotation error norm
    const char key = last_key_.load(std::memory_order_relaxed); // Load the latest teleop key value
    const bool teleop_present = this->count_publishers("/teleop/last_key") > 0; // Detect whether teleop is present
    const bool enabled_key = should_send_velocities(key); // Test whether the latest key requests motion
    const bool enabled = enabled_key && teleop_present; // Combine the motion enable conditions
    if (!has_converged_ && enabled) { // Latch convergence after the first valid start condition
      has_converged_ = true; // Store the convergence latch
      publish_convergence(true); // Publish the converged state when the latch changes
    }
    publish_zero_velocity(); // Keep the template robot output at zero velocity
    publish_errors(error_tr, error_rot); // Publish the current template error values
    if (!has_converged_) { // Publish false until the convergence latch is set
      publish_convergence(false); // Publish the unconverged state
    }
  }
};

int main(int argc, char** argv) { // Start the ROS process for the template robot node
  rclcpp::init(argc, argv); // Initialize ROS
  auto node = std::make_shared<TemplateRobotNode>(); // Create the template robot node
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3); // Create the original executor layout
  exec.add_node(node); // Add the node to the executor
  exec.spin(); // Spin the executor callbacks
  rclcpp::shutdown(); // Shut down ROS
  return 0; // Return a successful process code
}
