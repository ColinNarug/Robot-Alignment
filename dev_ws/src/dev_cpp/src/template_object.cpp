#include <rclcpp/rclcpp.hpp> // Include the ROS node api
#include <sensor_msgs/msg/image.hpp> // Include the image message type
#include <geometry_msgs/msg/transform_stamped.hpp> // Include the transform message type

#include <algorithm> // Include max helper functions
#include <chrono> // Include timer duration support
#include <functional> // Include function binding support
#include <mutex> // Include mutex support
#include <string> // Include string support

class TemplateObjectNode : public rclcpp::Node { // Define the template object node
public:
  TemplateObjectNode() : Node("template_object_node") { // Initialize the node with the original template name
    image_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the image callback group
    rclcpp::SubscriptionOptions cam_opts; // Create the image subscription options
    cam_opts.callback_group = image_handling_; // Assign the image callback group
    object_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Create the timer callback group
    width_ = this->declare_parameter<int>("width", 1920); // Store the requested image width
    height_ = this->declare_parameter<int>("height", 1080); // Store the requested image height
    fps_ = this->declare_parameter<int>("fps", 30); // Store the requested image rate
    active_camera_filename_ =
      this->declare_parameter<std::string>("active_camera_filename", "active_camera.yaml"); // Store the active camera file name
    qos_depth_ = this->declare_parameter<int>("qos_depth", 1); // Store the qos queue depth
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort"); // Store the qos reliability mode
    preferred_encoding_ =
      this->declare_parameter<std::string>("preferred_encoding", "bgr8"); // Store the preferred encoding label
    apriltag_threads_ = this->declare_parameter<int>("apriltag_threads", 2); // Store the worker thread parameter
    const int depth = std::max(1, qos_depth_); // Clamp the qos depth to a valid value
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth))); // Build the image qos profile
    qos.durability_volatile(); // Force volatile durability for streaming data
    if (qos_reliability_ == "reliable") { // Select reliable transport when requested
      qos.reliable(); // Apply reliable delivery
    } else { // Use the streaming default otherwise
      qos.best_effort(); // Apply best effort delivery
    }
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      qos,
      std::bind(&TemplateObjectNode::image_callback, this, std::placeholders::_1),
      cam_opts); // Subscribe to the camera image topic
    cMo_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("cMo", 10); // Create the pose publisher
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&TemplateObjectNode::publish_identity_transform, this),
      object_handling_); // Create the object publish timer
    RCLCPP_INFO(
      this->get_logger(),
      "template object node started width=%d height=%d fps=%d",
      width_,
      height_,
      fps_); // Report the template startup state
  }

private:
  rclcpp::CallbackGroup::SharedPtr image_handling_; // Store the image callback group
  rclcpp::CallbackGroup::SharedPtr object_handling_; // Store the timer callback group
  rclcpp::TimerBase::SharedPtr timer_; // Store the periodic timer handle
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Store the image subscription handle
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_pub_; // Store the pose publisher handle
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_; // Store the latest image pointer
  std::mutex image_mutex_; // Protect the shared image state
  bool have_image_{false}; // Track whether an image has been received
  int width_{1920}; // Store the width parameter
  int height_{1080}; // Store the height parameter
  int fps_{30}; // Store the frame rate parameter
  std::string active_camera_filename_{"active_camera.yaml"}; // Store the active camera file name
  int qos_depth_{1}; // Store the qos depth parameter
  std::string qos_reliability_{"best_effort"}; // Store the qos reliability parameter
  std::string preferred_encoding_{"bgr8"}; // Store the preferred encoding parameter
  int apriltag_threads_{2}; // Store the worker thread parameter

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) { // Save the latest image message
    if (!msg) { // Ignore null image pointers
      return; // Leave the callback when the message is empty
    }
    std::scoped_lock lock(image_mutex_); // Lock the shared image state
    latest_image_ = msg; // Store the latest image pointer
    have_image_ = true; // Mark the image stream as available
  }

  void publish_identity_transform() { // Publish a pose that preserves the object interface
    sensor_msgs::msg::Image::ConstSharedPtr image; // Store a local image pointer copy
    {
      std::scoped_lock lock(image_mutex_); // Lock the shared image state
      if (!have_image_ || !latest_image_) { // Wait until the camera stream exists
        return; // Leave the timer callback until an image arrives
      }
      image = latest_image_; // Copy the latest image pointer
    }


















///////////////////////////////////////////////////////
/*
Main development zone.
Use the image to perform object detection. 
*/
//////////////////////////////////////////////////////

















    geometry_msgs::msg::TransformStamped tf_msg; // Create the outgoing transform message
    tf_msg.header.stamp = image->header.stamp; // Forward the image timestamp
    tf_msg.header.frame_id = "camera_frame"; // Assign the parent frame id
    tf_msg.child_frame_id = "object_frame"; // Assign the child frame id
    tf_msg.transform.translation.x = 0.0; // Set the x translation to zero
    tf_msg.transform.translation.y = 0.0; // Set the y translation to zero
    tf_msg.transform.translation.z = 0.0; // Set the z translation to zero
    tf_msg.transform.rotation.x = 0.0; // Set the quaternion x value
    tf_msg.transform.rotation.y = 0.0; // Set the quaternion y value
    tf_msg.transform.rotation.z = 0.0; // Set the quaternion z value
    tf_msg.transform.rotation.w = 1.0; // Set the quaternion w value
    cMo_pub_->publish(tf_msg); // Publish the identity transform
  }
};

int main(int argc, char** argv) { // Start the ROS process for the template object node
  rclcpp::init(argc, argv); // Initialize ROS
  auto node = std::make_shared<TemplateObjectNode>(); // Create the template object node
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2); // Create the original executor layout
  exec.add_node(node); // Add the node to the executor
  exec.spin(); // Spin the executor callbacks
  rclcpp::shutdown(); // Shut down ROS
  return 0; // Return a successful process code
}
