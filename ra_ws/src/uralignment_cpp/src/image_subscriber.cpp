#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

class ImageViewerNode : public rclcpp::Node
{
public:
  ImageViewerNode()
  : Node("image_viewer_node")
  {
    topic_ = this->declare_parameter<std::string>("topic", "camera/color/image_raw");
    window_name_ = this->declare_parameter<std::string>("window_name", "Live Camera Feed");

    // Display controls
    display_rate_hz_ = this->declare_parameter<double>("display_rate_hz", 30.0); // GUI refresh rate
    scale_ = this->declare_parameter<double>("scale", 1.0); // 1.0 = full resolution
    quit_key_ = this->declare_parameter<int>("quit_key", 27); // ESC=27
    print_fps_ = this->declare_parameter<bool>("print_fps", true);

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

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_, qos,
      std::bind(&ImageViewerNode::image_callback, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

    if (display_rate_hz_ <= 0.0) display_rate_hz_ = 30.0;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / display_rate_hz_));

    timer_ = this->create_wall_timer(period, std::bind(&ImageViewerNode::display_tick, this));

    last_fps_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO_STREAM(this->get_logger(),
      "ImageViewer started:"
      << " topic=" << topic_
      << " qos_reliability=" << qos_reliability_
      << " qos_depth=" << qos_depth_
      << " display_rate_hz=" << display_rate_hz_
      << " scale=" << scale_
      << " preferred_encoding=" << preferred_encoding_);
  }
  ~ImageViewerNode() override {cv::destroyAllWindows();}

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    // Store latest message only (zero-copy)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_msg_ = msg;
    }
    // FPS stats (receive rate)
    if (print_fps_) {rx_count_++;}
  }

  void display_tick()
  {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      msg = last_msg_;
    }
    if (!msg) return;

    // Validate buffer size
    const size_t min_bytes = static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height);
    if (msg->data.size() < min_bytes) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Bad image buffer: data=" << msg->data.size() << " expected>=" << min_bytes
        << " (step=" << msg->step << ", height=" << msg->height << ")");
      return;
    }

    // Wrap the image using msg->step
    // Handle common encodings robustly
    cv::Mat view;
    cv::Mat bgr_for_display;

    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      view = cv::Mat(static_cast<int>(msg->height),
                     static_cast<int>(msg->width),
                     CV_8UC3,
                     const_cast<unsigned char*>(msg->data.data()),
                     static_cast<size_t>(msg->step));
      bgr_for_display = view;  // zero-copy
    }
    else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      view = cv::Mat(static_cast<int>(msg->height),
                     static_cast<int>(msg->width),
                     CV_8UC3,
                     const_cast<unsigned char*>(msg->data.data()),
                     static_cast<size_t>(msg->step));
      // Convert RGB->BGR for OpenCV display
      cv::cvtColor(view, bgr_for_display, cv::COLOR_RGB2BGR);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      view = cv::Mat(static_cast<int>(msg->height),
                     static_cast<int>(msg->width),
                     CV_8UC1,
                     const_cast<unsigned char*>(msg->data.data()),
                     static_cast<size_t>(msg->step));
      cv::cvtColor(view, bgr_for_display, cv::COLOR_GRAY2BGR);
    }
    else {
      // Unsupported/unknown encoding | fail fast instead of crashing later.
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported encoding: " << msg->encoding);
      return;
    }

    // Optional scaling for performance / screen fit
    cv::Mat shown;
    if (scale_ > 0.0 && scale_ != 1.0) {
      cv::resize(bgr_for_display, shown, cv::Size(), scale_, scale_, cv::INTER_AREA);
    } else {
      shown = bgr_for_display;
    }

    cv::imshow(window_name_, shown);

    const int key = cv::waitKey(1);
    if (key == quit_key_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Quit key pressed. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // Print receive FPS about once per second
    if (print_fps_) {
      const auto now = std::chrono::steady_clock::now();
      const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time_);
      if (dt.count() >= 1000) {
        const double secs = dt.count() / 1000.0;
        const double fps = rx_count_ / secs;
        RCLCPP_INFO_STREAM(this->get_logger(),
          "RX FPS: " << fps
          << " | last frame: " << msg->width << "x" << msg->height
          << " enc=" << msg->encoding << " step=" << msg->step);
        rx_count_ = 0;
        last_fps_time_ = now;
      }
    }
  }

  // Params
  std::string topic_;
  std::string window_name_;
  double display_rate_hz_{30.0};
  double scale_{1.0};
  int quit_key_{27};
  bool print_fps_{true};

  int qos_depth_{5};
  std::string qos_reliability_{"best_effort"};
  std::string preferred_encoding_{"bgr8"}; // kept for future extension

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest frame storage (zero-copy via shared_ptr)
  std::mutex mtx_;
  sensor_msgs::msg::Image::ConstSharedPtr last_msg_;

  // FPS stats
  std::atomic<uint64_t> rx_count_{0};
  std::chrono::steady_clock::time_point last_fps_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageViewerNode>());
  rclcpp::shutdown();
  return 0;
}
