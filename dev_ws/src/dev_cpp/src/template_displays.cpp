#include <rclcpp/rclcpp.hpp> // Include the ROS node api
#include <sensor_msgs/msg/image.hpp> // Include the image message type
#include <std_msgs/msg/float64_multi_array.hpp> // Include the numeric array message type
#include <geometry_msgs/msg/transform_stamped.hpp> // Include the transform message type

#include <chrono> // Include timer duration support
#include <mutex> // Include mutex support
#include <string> // Include string support

class DisplayTemplateNode : public rclcpp::Node { // Define the template displays node
public:
  DisplayTemplateNode() : Node("displays_node") { // Initialize the node with the original interface name
    image_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the image callback group
    rclcpp::SubscriptionOptions cam_opts; // Create the image subscription options
    cam_opts.callback_group = image_handling_; // Assign the image callback group
    vector_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Create the vector callback group
    rclcpp::SubscriptionOptions vector_opts; // Create the vector subscription options
    vector_opts.callback_group = vector_handling_; // Assign the vector callback group
    plot_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the timer callback group
    width_ = this->declare_parameter<int>("width", 1920); // Store the requested image width
    height_ = this->declare_parameter<int>("height", 1080); // Store the requested image height
    fps_ = this->declare_parameter<int>("fps", 30); // Store the requested frame rate
    qos_depth_ = this->declare_parameter<int>("qos_depth", 5); // Store the qos queue depth
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort"); // Store the qos reliability mode
    preferred_encoding_ =
      this->declare_parameter<std::string>("preferred_encoding", "bgr8"); // Store the preferred encoding label
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth_))); // Build the image qos profile
    qos.durability_volatile(); // Force volatile durability for streaming data
    if (qos_reliability_ == "reliable") { // Select reliable transport when requested
      qos.reliable(); // Apply reliable delivery
    } else { // Use the streaming default otherwise
      qos.best_effort(); // Apply best effort delivery
    }
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      qos,
      std::bind(&DisplayTemplateNode::image_callback, this, std::placeholders::_1),
      cam_opts); // Subscribe to the image topic
    cMo_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "cMo",
      10,
      std::bind(&DisplayTemplateNode::cMo_callback, this, std::placeholders::_1),
      vector_opts); // Subscribe to the pose topic
    v_c_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "v_c",
      10,
      std::bind(&DisplayTemplateNode::v_c_callback, this, std::placeholders::_1),
      vector_opts); // Subscribe to the velocity topic
    errors_xyz_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "errors_xyz",
      10,
      std::bind(&DisplayTemplateNode::errors_xyz_callback, this, std::placeholders::_1),
      vector_opts); // Subscribe to the component error topic
    error_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "error",
      10,
      std::bind(&DisplayTemplateNode::error_callback, this, std::placeholders::_1),
      vector_opts); // Subscribe to the error norm topic
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&DisplayTemplateNode::plot_callback, this),
      plot_handling_); // Create the periodic template timer
    RCLCPP_INFO(this->get_logger(), "Display template node started"); // Report the template startup state
  }

private:
  rclcpp::CallbackGroup::SharedPtr image_handling_; // Store the image callback group
  rclcpp::CallbackGroup::SharedPtr vector_handling_; // Store the vector callback group
  rclcpp::CallbackGroup::SharedPtr plot_handling_; // Store the timer callback group
  rclcpp::TimerBase::SharedPtr timer_; // Store the periodic timer handle
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Store the image subscription handle
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_sub_; // Store the pose subscription handle
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_sub_; // Store the velocity subscription handle
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr errors_xyz_sub_; // Store the component error subscription handle
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr error_sub_; // Store the error norm subscription handle
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_; // Store the latest image pointer
  geometry_msgs::msg::TransformStamped latest_cMo_; // Store the latest object transform
  std_msgs::msg::Float64MultiArray latest_v_c_; // Store the latest velocity vector
  std_msgs::msg::Float64MultiArray latest_errors_xyz_; // Store the latest component error vector
  std_msgs::msg::Float64MultiArray latest_error_; // Store the latest error norm vector
  std::mutex image_mtx_; // Protect the shared image state
  std::mutex state_mtx_; // Protect the shared vector state
  bool have_image_{false}; // Track whether image data exists
  bool have_cMo_{false}; // Track whether pose data exists
  bool have_v_c_{false}; // Track whether velocity data exists
  bool have_errors_xyz_{false}; // Track whether component error data exists
  bool have_error_{false}; // Track whether error norm data exists
  size_t timer_count_{0}; // Count timer callback executions
  size_t image_count_{0}; // Count received images
  size_t cMo_count_{0}; // Count received poses
  size_t vector_count_{0}; // Count received vector messages
  int width_{1920}; // Store the width parameter
  int height_{1080}; // Store the height parameter
  int fps_{30}; // Store the frame rate parameter
  int qos_depth_{5}; // Store the qos depth parameter
  std::string qos_reliability_{"best_effort"}; // Store the qos reliability parameter
  std::string preferred_encoding_{"bgr8"}; // Store the preferred encoding parameter

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) { // Save the latest image message
    if (!msg) { // Ignore null image pointers
      return; // Leave the callback when the message is empty
    }
    std::scoped_lock lock(image_mtx_); // Lock the shared image state
    latest_image_ = msg; // Copy the latest image pointer
    have_image_ = true; // Mark the image stream as available
    image_count_++; // Count the received image
  }

  void cMo_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) { // Save the latest object pose
    if (!msg) { // Ignore null transform pointers
      return; // Leave the callback when the message is empty
    }
    std::scoped_lock lock(state_mtx_); // Lock the shared vector state
    latest_cMo_ = *msg; // Copy the latest object transform
    have_cMo_ = true; // Mark the pose stream as available
    cMo_count_++; // Count the received pose message
  }

  void v_c_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) { // Save the latest velocity message
    if (!msg) { // Ignore null array pointers
      return; // Leave the callback when the message is empty
    }
    std::scoped_lock lock(state_mtx_); // Lock the shared vector state
    latest_v_c_ = *msg; // Copy the latest velocity vector
    have_v_c_ = true; // Mark the velocity stream as available
    vector_count_++; // Count the received vector message
  }

  void errors_xyz_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) { // Save the latest component error message
    if (!msg) { // Ignore null array pointers
      return; // Leave the callback when the message is empty
    }
    std::scoped_lock lock(state_mtx_); // Lock the shared vector state
    latest_errors_xyz_ = *msg; // Copy the latest component error vector
    have_errors_xyz_ = true; // Mark the component error stream as available
    vector_count_++; // Count the received vector message
  }

  void error_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) { // Save the latest error norm message
    if (!msg) { // Ignore null array pointers
      return; // Leave the callback when the message is empty
    }
    std::scoped_lock lock(state_mtx_); // Lock the shared vector state
    latest_error_ = *msg; // Copy the latest error norm vector
    have_error_ = true; // Mark the error norm stream as available
    vector_count_++; // Count the received vector message
  }

  void plot_callback() { // Run the template displays timer work
    bool have_image_local = false; // Store a local copy of the image state
    bool have_cMo_local = false; // Store a local copy of the pose state
    bool have_v_c_local = false; // Store a local copy of the velocity state
    bool have_errors_xyz_local = false; // Store a local copy of the component error state
    bool have_error_local = false; // Store a local copy of the error norm state
    {
      std::scoped_lock lock(image_mtx_); // Lock the shared image state
      have_image_local = have_image_; // Copy the image availability flag
    }
    {
      std::scoped_lock lock(state_mtx_); // Lock the shared vector state
      have_cMo_local = have_cMo_; // Copy the pose availability flag
      have_v_c_local = have_v_c_; // Copy the velocity availability flag
      have_errors_xyz_local = have_errors_xyz_; // Copy the component error availability flag
      have_error_local = have_error_; // Copy the error norm availability flag
    }
    timer_count_++; // Count the timer callback execution
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Display template active image %s cMo %s vc %s errorsxyz %s error %s timer %zu imagecount %zu cMocount %zu vectorcount %zu",
      have_image_local ? "yes" : "no",
      have_cMo_local ? "yes" : "no",
      have_v_c_local ? "yes" : "no",
      have_errors_xyz_local ? "yes" : "no",
      have_error_local ? "yes" : "no",
      timer_count_,
      image_count_,
      cMo_count_,
      vector_count_); // Report the template status summary














  ///////////////////////////////////////////////////////
  /*
  Main development zone.
  Plot/Graph information of interest
  */
  ///////////////////////////////////////////////////////



















  }
};

int main(int argc, char **argv) { // Start the ROS process for the template displays node
  rclcpp::init(argc, argv); // Initialize ROS
  auto node = std::make_shared<DisplayTemplateNode>(); // Create the template displays node
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3); // Create the original executor layout
  exec.add_node(node); // Add the node to the executor
  exec.spin(); // Spin the executor callbacks
  rclcpp::shutdown(); // Shut down ROS
  return 0; // Return a successful process code
}
