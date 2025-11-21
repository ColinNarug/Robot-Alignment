#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

// Define the camera node class:
class D435iCameraNode : public rclcpp::Node
{
public:
  // Constructor
  D435iCameraNode()
  : Node("d435i_camera_node") // Name the ROS2 node
  {
    // Configure the RealSense pipeline:
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_BGR8, 30); // Enable BGR stream
    pipe_.start(cfg); // Start the RealSense pipeline
    
  }

  void post_init()
  {
    // Create an ImageTransport object scoped to this constructor only:
    image_transport::ImageTransport it(shared_from_this());
    pub_ = it.advertise("camera/color/image_raw", 10); // Advertise image topic

    // Create a timer to call timer_callback() every ~33ms (~30 FPS):
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&D435iCameraNode::timer_callback, this)
    );
  }

private:
  // Callback to capture and publish camera frames:
  void timer_callback()
  {
    // Wait for the next set of frames
    rs2::frameset frames = pipe_.wait_for_frames();
    rs2::video_frame color_frame = frames.get_color_frame(); // Get color frame

    if (!color_frame) return; // Skip if no frame available

    // Convert RealSense frame to OpenCV matrix:
    cv::Mat color(
      cv::Size(960, 540), // Match resolution
      CV_8UC3, // 8-bit 3-channel image
      (void*)color_frame.get_data(), // Image data pointer
      cv::Mat::AUTO_STEP   // Auto-stride
    );

    // Create ROS2 message header:
    std_msgs::msg::Header header;
    header.stamp = this->now(); // Timestamp
    header.frame_id = "camera_link"; // Frame ID

    // Convert OpenCV image to ROS2 Image message:
    auto msg = cv_bridge::CvImage(header, "bgr8", color).toImageMsg();

    pub_.publish(msg); // Publish the message
  }

  rs2::pipeline pipe_; // RealSense pipeline object
  image_transport::Publisher pub_; // Publisher for image topic
  rclcpp::TimerBase::SharedPtr timer_; // Timer for publishing frames
};

// Main function to start the node:
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); // Initialize ROS2
  auto node = std::make_shared<D435iCameraNode>();
  node->post_init();
  rclcpp::spin(node); // Run the node
  rclcpp::shutdown(); // Cleanup after node is shut down
  return 0;
}