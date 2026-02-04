#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <librealsense2/rs.hpp>

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

class D435iCameraNode : public rclcpp::Node
{
public:
  explicit D435iCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("d435i_camera_node", options)
  {
    // Parameters
    width_           = this->declare_parameter<int>("width", 1920);
    height_          = this->declare_parameter<int>("height", 1080);
    fps_             = this->declare_parameter<int>("fps", 30);
    frame_id_        = this->declare_parameter<std::string>("frame_id", "camera_link");
    qos_depth_       = this->declare_parameter<int>("qos_depth", 1);
    qos_reliability_ = this->declare_parameter<std::string>("qos_reliability", "best_effort"); // "reliable" or "best_effort"
    encoding_        = this->declare_parameter<std::string>("encoding", "bgr8");

    // QoS: for video, prefer best_effort + depth 1 (latest frame wins)
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth_)))};
    qos.durability_volatile();
    if (qos_reliability_ == "best_effort") qos.best_effort();
    else qos.reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", qos);

    // Pre-size message pool
    const size_t nominal_stride = static_cast<size_t>(width_) * 3; // bgr8/rgb8
    const size_t nominal_bytes  = static_cast<size_t>(height_) * nominal_stride;

    for (auto & msg : pool_) {
      msg.header.frame_id = frame_id_;
      msg.is_bigendian = false;
      msg.width  = static_cast<uint32_t>(width_);
      msg.height = static_cast<uint32_t>(height_);
      msg.encoding = encoding_;
      msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(nominal_stride);
      msg.data.resize(nominal_bytes);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "D435i Camera Node configured. %dx%d @ %d FPS. QoS: %s depth=%d encoding=%s",
      width_, height_, fps_, qos_reliability_.c_str(), qos_depth_, encoding_.c_str());
  }

  void start()
  {
    running_.store(true);

    // Start RealSense pipeline
    rs2::config cfg;
    if (encoding_ == "rgb8") {
      cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);
    } else {
      // default
      cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
    }
    pipe_.start(cfg);

    capture_thread_ = std::thread(&D435iCameraNode::captureLoop, this);
    publish_thread_ = std::thread(&D435iCameraNode::publishLoop, this);

    RCLCPP_INFO(this->get_logger(), "D435i Camera Node started threads.");
  }

  void stop()
  {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) return;
    // Wake publisher if it is sleeping:
    {
      std::lock_guard<std::mutex> lk(m_);
      shutting_down_ = true;
    }
    cv_.notify_all();
    // Stopping pipeline will unblock wait_for_frames():
    try { pipe_.stop(); } catch (...) {}

    if (capture_thread_.joinable()) capture_thread_.join();
    if (publish_thread_.joinable()) publish_thread_.join();
  }

  ~D435iCameraNode() override { stop(); }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rs2::pipeline pipe_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::thread publish_thread_;

  // Params
  int width_{1920};
  int height_{1080};
  int fps_{30};
  std::string frame_id_{"camera_link"};
  int qos_depth_{1};
  std::string qos_reliability_{"best_effort"};
  std::string encoding_{"bgr8"};

  // 3-slot pool to allow capture+publish overlap
  static constexpr size_t kPoolSize = 3;

  enum class SlotState { FREE, WRITING, READY, INFLIGHT };

  std::array<sensor_msgs::msg::Image, kPoolSize> pool_;
  std::array<SlotState, kPoolSize> state_{
    SlotState::FREE, SlotState::FREE, SlotState::FREE
  };

  std::mutex m_;
  std::condition_variable cv_;
  bool shutting_down_{false};
  size_t latest_ready_idx_{0};
  bool has_ready_{false};

  void captureLoop()
  {
    while (rclcpp::ok() && running_.load())
    {
      try
      {
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        if (!color_frame) continue;

        const int w = color_frame.get_width();
        const int h = color_frame.get_height();
        const int stride = color_frame.get_stride_in_bytes();
        const size_t frame_bytes = static_cast<size_t>(h) * static_cast<size_t>(stride);

        // Reserve a FREE slot (never block the camera thread)
        size_t idx = kPoolSize;
        {
          std::lock_guard<std::mutex> lk(m_);
          for (size_t i = 0; i < kPoolSize; ++i) {
            if (state_[i] == SlotState::FREE) { idx = i; break; }
          }
          if (idx == kPoolSize) {
            // No free slot? -> drop this frame immediately
            continue;
          }
          state_[idx] = SlotState::WRITING;
        }

        // Fill the message outside the lock
        auto & msg = pool_[idx];
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        msg.width  = static_cast<uint32_t>(w);
        msg.height = static_cast<uint32_t>(h);
        msg.encoding = encoding_;
        msg.is_bigendian = false;
        msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(stride);
        if (msg.data.size() != frame_bytes) msg.data.resize(frame_bytes);

        std::memcpy(msg.data.data(), color_frame.get_data(), frame_bytes);

        // Mark READY and signal publisher
        {
          std::lock_guard<std::mutex> lk(m_);
          state_[idx] = SlotState::READY;
          latest_ready_idx_ = idx;
          has_ready_ = true;
        }
        cv_.notify_one();
      }
      catch (const rs2::error&) { if (!running_.load()) break; }
      catch (...)             { if (!running_.load()) break; }
    }
  }

  void publishLoop()
  {
    while (rclcpp::ok() && running_.load())
    {
      size_t idx = kPoolSize;

      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] {
          return shutting_down_ || (has_ready_ && anyReadyUnsafe());
        });
        if (shutting_down_ || !running_.load()) break;

        // Publish the latest READY frame; drop older READY frames
        idx = latest_ready_idx_;
        if (idx >= kPoolSize || state_[idx] != SlotState::READY) {
          // Fallback: find any READY
          for (size_t i = 0; i < kPoolSize; ++i) {
            if (state_[i] == SlotState::READY) { idx = i; break; }
          }
        }
        if (idx == kPoolSize) {
          has_ready_ = false;
          continue;
        }

        // Drop older ready frames
        for (size_t i = 0; i < kPoolSize; ++i) {
          if (i != idx && state_[i] == SlotState::READY) state_[i] = SlotState::FREE;
        }

        state_[idx] = SlotState::INFLIGHT;
        has_ready_ = anyReadyUnsafe();
      }

      // Publish outside the lock
      pub_->publish(pool_[idx]);

      // Free the slot
      {
        std::lock_guard<std::mutex> lk(m_);
        state_[idx] = SlotState::FREE;
        has_ready_ = anyReadyUnsafe();
      }
    }
  }

  bool anyReadyUnsafe() const
  {
    for (auto s : state_) if (s == SlotState::READY) return true;
    return false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<D435iCameraNode>(options);
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
