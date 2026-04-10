#include <rclcpp/rclcpp.hpp> // Include the ROS node api
#include <std_msgs/msg/bool.hpp> // Include the bool message type
#include <std_msgs/msg/char.hpp> // Include the char message type
#include <geometry_msgs/msg/wrench_stamped.hpp> // Include the wrench message type
#include <geometry_msgs/msg/twist_stamped.hpp> // Include the twist message type

#include <chrono> // Include timer duration support
#include <mutex> // Include mutex support
#include <string> // Include string support

using namespace std::chrono_literals; // Enable millisecond duration literals

class OffsetTemplateNode : public rclcpp::Node { // Define the template offset node
public:
  OffsetTemplateNode() : Node("offset_node") { // Initialize the node with the original interface name
    f_contact_ = this->declare_parameter<double>("f_contact", 5.0); // Store the contact force parameter
    debounce_n_ = this->declare_parameter<int>("debounce_n", 6); // Store the debounce count parameter
    v0_ = this->declare_parameter<double>("v0", 0.003); // Store the nominal approach speed parameter
    vmin_ = this->declare_parameter<double>("vmin", 0.001); // Store the minimum speed parameter
    tau_ = this->declare_parameter<double>("tau", 1.0); // Store the ramp time parameter
    timeout_s_ = this->declare_parameter<double>("approach_timeout_s", 180.0); // Store the timeout parameter
    contact_grace_s_ = this->declare_parameter<double>("contact_grace_s", 0.25); // Store the contact grace parameter
    baseline_window_s_ =
      this->declare_parameter<double>("baseline_window_s", 0.25); // Store the baseline window parameter
    wrench_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the wrench callback group
    offset_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the state callback group
    rclcpp::SubscriptionOptions offset_opts; // Create the subscription options
    offset_opts.callback_group = offset_handling_; // Assign the state callback group
    sub_conv_ = this->create_subscription<std_msgs::msg::Bool>(
      "/pbvs/converged",
      10,
      std::bind(&OffsetTemplateNode::convergence_callback, this, std::placeholders::_1),
      offset_opts); // Subscribe to the convergence topic
    rclcpp::QoS key_qos(1); // Build the latched teleop qos profile
    key_qos.reliable().transient_local(); // Match the original teleop qos settings
    sub_key_ = this->create_subscription<std_msgs::msg::Char>(
      "/teleop/last_key",
      key_qos,
      std::bind(&OffsetTemplateNode::key_callback, this, std::placeholders::_1),
      offset_opts); // Subscribe to the teleop topic
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/offset/cmd_twist",
      rclcpp::SensorDataQoS()); // Create the offset twist publisher
    wrench_timer_ = this->create_wall_timer(
      8ms,
      std::bind(&OffsetTemplateNode::wrench_callback, this),
      wrench_handling_); // Create the synthetic wrench timer
    offset_timer_ = this->create_wall_timer(
      8ms,
      std::bind(&OffsetTemplateNode::tick, this),
      offset_handling_); // Create the state machine timer
    RCLCPP_INFO(this->get_logger(), "Offset template node started"); // Report the template startup state
  }

private:
  enum class State { IDLE, ACTIVE }; // Store the minimal template states

  State state_{State::IDLE}; // Store the current template state
  rclcpp::CallbackGroup::SharedPtr wrench_handling_; // Store the wrench callback group
  rclcpp::CallbackGroup::SharedPtr offset_handling_; // Store the state callback group
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_conv_; // Store the convergence subscription handle
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr sub_key_; // Store the teleop subscription handle
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_; // Store the twist publisher handle
  rclcpp::TimerBase::SharedPtr wrench_timer_; // Store the synthetic wrench timer handle
  rclcpp::TimerBase::SharedPtr offset_timer_; // Store the state machine timer handle
  mutable std::mutex wrench_mtx_; // Protect the shared wrench state
  geometry_msgs::msg::WrenchStamped last_wrench_; // Store the latest synthetic wrench message
  rclcpp::Time last_wrench_rx_time_; // Store the latest wrench timestamp
  bool pbvs_converged_{false}; // Track the convergence state
  bool haptics_enabled_{false}; // Track whether the offset logic is enabled
  bool have_wrench_{false}; // Track whether a wrench sample exists
  bool printed_first_wrench_{false}; // Track whether the first wrench log was sent
  size_t tick_count_{0}; // Count timer callback executions
  double f_contact_{5.0}; // Store the contact force parameter
  int debounce_n_{6}; // Store the debounce parameter
  double v0_{0.003}; // Store the nominal speed parameter
  double vmin_{0.001}; // Store the minimum speed parameter
  double tau_{1.0}; // Store the ramp time parameter
  double timeout_s_{180.0}; // Store the timeout parameter
  double contact_grace_s_{0.25}; // Store the contact grace parameter
  double baseline_window_s_{0.25}; // Store the baseline window parameter

  void convergence_callback(const std_msgs::msg::Bool::SharedPtr msg) { // Save the latest convergence input
    if (!msg) { // Ignore null convergence pointers
      return; // Leave the callback when the message is empty
    }
    pbvs_converged_ = msg->data; // Store the convergence state
  }

  void wrench_callback() { // Generate a synthetic zero wrench sample
    std::lock_guard<std::mutex> lock(wrench_mtx_); // Lock the shared wrench state
    const rclcpp::Time stamp = this->now(); // Capture the current time
    last_wrench_rx_time_ = stamp; // Store the wrench timestamp
    last_wrench_.header.stamp = stamp; // Stamp the synthetic wrench message
    last_wrench_.header.frame_id = "base"; // Assign a stable wrench frame id
    last_wrench_.wrench.force.x = 0.0; // Set the x force value to zero
    last_wrench_.wrench.force.y = 0.0; // Set the y force value to zero
    last_wrench_.wrench.force.z = 0.0; // Set the z force value to zero
    last_wrench_.wrench.torque.x = 0.0; // Set the x torque value to zero
    last_wrench_.wrench.torque.y = 0.0; // Set the y torque value to zero
    last_wrench_.wrench.torque.z = 0.0; // Set the z torque value to zero
    have_wrench_ = true; // Mark the wrench stream as available
    if (!printed_first_wrench_) { // Report the synthetic wrench source only once
      printed_first_wrench_ = true; // Mark the first wrench log as complete
      RCLCPP_INFO(this->get_logger(), "Offset template synthetic wrench active"); // Report the synthetic wrench source
    }
  }

  void key_callback(const std_msgs::msg::Char::SharedPtr msg) { // Process the teleop key input
    if (!msg) { // Ignore null key pointers
      return; // Leave the callback when the message is empty
    }
    const char k = static_cast<char>(msg->data); // Copy the received key value
    if (k == 'x' || k == 'X') { // Handle the enable command
      haptics_enabled_ = true; // Enable the template offset logic
      RCLCPP_INFO(this->get_logger(), "Offset template enabled"); // Report the enable event
    } else if (k == ' ' || k == 'q' || k == 'Q' || k == 0x1B) { // Handle the disable style commands
      haptics_enabled_ = false; // Disable the template offset logic
      state_ = State::IDLE; // Return the state machine to idle
      publish_zero(this->now()); // Publish a zero twist command immediately
      RCLCPP_WARN(this->get_logger(), "Offset template disabled"); // Report the disable event
    }
  }

  void tick() { // Run the minimal template state machine
    tick_count_++; // Count the timer callback execution
    bool have_wrench_local = false; // Store a local copy of the wrench state
    {
      std::lock_guard<std::mutex> lock(wrench_mtx_); // Lock the shared wrench state
      have_wrench_local = have_wrench_; // Copy the wrench availability flag
    }
    if (!haptics_enabled_ || !pbvs_converged_ || !have_wrench_local) { // Hold the idle conditions
      state_ = State::IDLE; // Keep the state machine in idle
      publish_zero(this->now()); // Publish a zero twist command
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Offset template idle enabled %s converged %s wrench %s ticks %zu",
        haptics_enabled_ ? "yes" : "no",
        pbvs_converged_ ? "yes" : "no",
        have_wrench_local ? "yes" : "no",
        tick_count_); // Report the idle status summary
      return; // Leave the timer callback after idle handling
    }
    state_ = State::ACTIVE; // Mark the state machine as active



















  ///////////////////////////////////////////////////////
  /*
  Main development zone.
  Define function for approach velocity.
  Ex. v = v_target * (1.0 - std::exp(-t / tau_));
  */
  ///////////////////////////////////////////////////////






















    publish_zero(this->now()); // Keep the template offset output at zero twist
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Offset template active and publishing zero twist"); // Report the active status summary
  }

  void publish_zero(const rclcpp::Time& stamp) { // Publish a zero twist command for template safety
    geometry_msgs::msg::TwistStamped cmd; // Create the outgoing twist message
    cmd.header.stamp = stamp; // Stamp the outgoing message
    cmd.header.frame_id = "tooling_frame"; // Assign the original command frame id
    pub_twist_->publish(cmd); // Publish the zero twist message
  }
};

int main(int argc, char **argv) { // Start the ROS process for the template offset node
  rclcpp::init(argc, argv); // Initialize ROS
  auto node = std::make_shared<OffsetTemplateNode>(); // Create the template offset node
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2); // Create the original executor layout
  exec.add_node(node); // Add the node to the executor
  exec.spin(); // Spin the executor callbacks
  rclcpp::shutdown(); // Shut down ROS
  return 0; // Return a successful process code
}
