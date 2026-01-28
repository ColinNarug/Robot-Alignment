#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <librealsense2/rs.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>


// Define the camera node class:
class D435iCameraNode : public rclcpp::Node
{
public:
  // Constructor
  explicit D435iCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("d435i_camera_node", options)
  {
    // Parameters
    width_ = this->declare_parameter<int>("width",1920);
    height_ = this->declare_parameter<int>("height",1080);
    fps_ = this->declare_parameter<int>("fps",30);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_link");
    qos_depth_ = this->declare_parameter<int>("qos_depth", 10);
    qos_reliability_ = this-> declare_parameter<std::string>("qos_reliability", "reliable"); // "reliable" or "best_effort"
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
    qos.durability_volatile();
    if(qos_reliability_ == "best_effort")
    {
      qos.best_effort();
    }
    else
    {
      qos.reliable();
    }
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", qos);
    rs2::config cfg; // Configure the RealSense pipeline:
    cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_); // Enable BGR stream
    pipe_.start(cfg); // Start the RealSense pipeline
    RCLCPP_INFO(this->get_logger(), "D435i Camera Node Started. Resolution: %dx%d @ %d FPS. QoS: %s. Depth: %d.",
      width_, height_, fps_, qos_reliability_.c_str(), qos_depth_);
  }

  void start()
  {
    running_.store(true);
    capture_thread_ = std::thread(&D435iCameraNode::capture, this);
  }
  void stop()
  {
    bool expected = true;
    if(!running_.compare_exchange_strong(expected, false)) {return;}
    try
    {
      pipe_.stop();
    }catch(...){}
    if (capture_thread_.joinable()) {capture_thread_.join();}
  }
  ~D435iCameraNode() override {stop();}

  private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rs2::pipeline pipe_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  // Params:
  int width_{1920};
  int height_{1080};
  int fps_{30};
  std::string frame_id_{"camera_link"};
  int qos_depth_{10};
  std::string qos_reliability_{"reliable"};
  std::string encoding_{"bgr8"};

  void capture() // Callback to capture and publish camera frames:
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id_;
    msg.is_bigendian = false;

    while(rclcpp::ok() && running_.load())
    {
      try
      {
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        if(!color_frame) {continue;}
        const int w = color_frame.get_width();
        const int h = color_frame.get_height();
        const int stride = color_frame.get_stride_in_bytes();
        const size_t frame_bytes = static_cast<size_t>(h) * static_cast<size_t>(stride);
        msg.header.stamp = this->now();
        msg.width = static_cast<uint32_t>(w);
        msg.height = static_cast<uint32_t>(h);
        msg.encoding = encoding_;
        msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(stride);
        if(msg.data.size() != frame_bytes)
        {
          msg.data.resize(frame_bytes);
        }
        std::memcpy(msg.data.data(), color_frame.get_data(), frame_bytes);
        pub_->publish(msg);
      }
      catch(const rs2::error&) {if(!running_.load()) break;}
      catch(...) {if(!running_.load()) break;}
    }
  };
};

// Main function to start the node:
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); // Initialize ROS2
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<D435iCameraNode>(options);
  node->start();
  rclcpp::spin(node); // Run the node
  rclcpp::shutdown(); // Cleanup after node is shut down
  return 0;
}