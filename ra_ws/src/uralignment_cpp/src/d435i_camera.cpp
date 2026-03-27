#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <librealsense2/rs.hpp>
#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <unordered_set>

namespace fs = std::filesystem;

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
    serial_ = this->declare_parameter<std::string>("serial", "");
    active_camera_filename_ = this->declare_parameter<std::string>("active_camera_filename", "active_camera.yaml");
    config_subdir_          = this->declare_parameter<std::string>("config_subdir", "config");
    mirror_packages_ = this->declare_parameter<std::vector<std::string>>(
      "mirror_packages", std::vector<std::string>{"uralignment_cpp", "calibration_cpp", "uralignment_py"});
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
      "D435i Camera Node configured. req=%dx%d @ %d FPS. QoS: %s depth=%d encoding=%s serial=%s",
      width_, height_, fps_, qos_reliability_.c_str(), qos_depth_, encoding_.c_str(), serial_.c_str());
  }

  void start()
  {
    running_.store(true);

    // Start RealSense pipeline
    rs2::config cfg;
    if (!serial_.empty()) 
    {
      cfg.enable_device(serial_);
    }
    if (encoding_ == "rgb8") {
      cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);
    } 
    else 
    {
      cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_); // default
    }
    rs2::pipeline_profile profile = pipe_.start(cfg);

    // Determine actual serial + actual stream size (may differ from requested)
    try {
      auto dev = profile.get_device();
      actual_serial_ = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    } catch (...) {
      actual_serial_.clear();
    }

    int aw = width_, ah = height_;
    try {
      auto sp = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      aw = sp.width();
      ah = sp.height();
    } catch (...) {}

    RCLCPP_INFO(this->get_logger(),
      "RealSense active device: actual_serial=%s (param serial=%s), stream=%dx%d, fps=%d",
      actual_serial_.c_str(), serial_.c_str(), aw, ah, fps_);

    write_active_camera_yaml(actual_serial_, aw, ah);
    capture_thread_ = std::thread(&D435iCameraNode::captureLoop, this);
    publish_thread_ = std::thread(&D435iCameraNode::publishLoop, this);

    RCLCPP_INFO(this->get_logger(), "D435i Camera Node started threads.");
  }

  void stop()
  {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) return;
    // Wake publisher if sleeping:
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

  ~D435iCameraNode() override { stop(); } // Begin d435i camera node

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
  std::string serial_;
  std::string actual_serial_;

  std::string active_camera_filename_{"active_camera.yaml"};
  std::string config_subdir_{"config"};
  std::vector<std::string> mirror_packages_;

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

  // Active-camera YAML. Derive the workspace root by trimming the install path off a package share path
  static std::string infer_workspace_root_from_share(const std::string& share_dir) 
  {
    const std::string needle = "/install/";
    const auto pos = share_dir.find(needle);
    if (pos == std::string::npos) return "";
    return share_dir.substr(0, pos);
  }

  // Deduplicate directory strings while preserving order
  std::vector<std::string> unique_dirs_(const std::vector<std::string>& in) const
  {
    std::unordered_set<std::string> seen;
    std::vector<std::string> out;
    out.reserve(in.size());
    for (const auto& s : in) 
    {
      if (s.empty()) continue;
      if (seen.insert(s).second) out.push_back(s);
    }
    return out;
  }

  // Build the list of congiruation directories that should get active_camera.yaml
  std::vector<std::string> compute_active_camera_target_dirs() const
  {
    std::vector<std::string> dirs;

    // Always write to each package's install/share config dir (runtime-safe)
    for (const auto& pkg : mirror_packages_) 
    {
      try 
      {
        const std::string share = ament_index_cpp::get_package_share_directory(pkg);
        dirs.push_back(share + "/" + config_subdir_);
      } 
      catch (...) {}
    }

    // Also write to workspace-visible locations
    std::string ws_root;
    try {
      const std::string share = ament_index_cpp::get_package_share_directory("uralignment_cpp");
      ws_root = infer_workspace_root_from_share(share);
    } catch (...) {}

    if (!ws_root.empty()) {
      dirs.push_back(ws_root + "/config");
      for (const auto& pkg : mirror_packages_) {
        dirs.push_back(ws_root + "/src/" + pkg + "/" + config_subdir_);
        dirs.push_back(ws_root + "/install/" + pkg + "/share/" + pkg + "/" + config_subdir_);
      }
    }

    return unique_dirs_(dirs);
  }

  // Write active camera serial and resolution
  bool write_active_camera_yaml(const std::string& serial, int w, int h)
  {
    if (serial.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "Active camera serial is empty; active_camera.yaml will still be written but other nodes may not resolve intrinsics filenames.");
    }

    const auto dirs = compute_active_camera_target_dirs();
    if (dirs.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No target directories available to write %s.", active_camera_filename_.c_str());
      return false;
    }

    bool any_ok = false;
    for (const auto& dir : dirs)
    {
      try {
        fs::create_directories(dir);
        const std::string path = dir + "/" + active_camera_filename_;

        cv::FileStorage fsw(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
        if (!fsw.isOpened()) {
          RCLCPP_WARN(this->get_logger(), "Could not open for write: %s", path.c_str());
          continue;
        }

        fsw << "serial" << serial;
        fsw << "width"  << w;
        fsw << "height" << h;
        fsw.release();

        any_ok = true;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed writing active_camera.yaml to %s: %s", dir.c_str(), e.what());
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Failed writing active_camera.yaml to %s: unknown error", dir.c_str());
      }
    }

    if (any_ok) {
      RCLCPP_INFO(this->get_logger(),
        "Wrote %s (serial=%s, %dx%d) to %zu config locations.",
        active_camera_filename_.c_str(), serial.c_str(), w, h, dirs.size());
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to write %s to any target location.", active_camera_filename_.c_str());
    }
    return any_ok;
  }

  // Pull frames from the RealSense pipline and fill free slots
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
      catch (...) { if (!running_.load()) break; }
    }
  }

  // Publish the newest available image frame
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

      pub_->publish(pool_[idx]);

      // Free the slot
      {
        std::lock_guard<std::mutex> lk(m_);
        state_[idx] = SlotState::FREE;
        has_ready_ = anyReadyUnsafe();
      }
    }
  }

  // Scan the slot state for any ready entry while the mutex is already held
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
