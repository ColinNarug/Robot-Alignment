#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core/persistence.hpp>
#include <filesystem>
#include "calibration_cpp/terminal_io.hpp"
#include "calibration_cpp/transform_math.hpp"
#include "calibration_cpp/yaml_io.hpp"
#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <deque>
#include <condition_variable>
#include <limits>
#include <cstdint>

namespace fs = std::filesystem;

// Search target directories and read first valid active_camera.yaml
static bool read_active_camera_yaml_from_targets(
  const calibration_cpp::WriteTargets& targets,
  std::string& out_serial,
  int& out_width,
  int& out_height,
  std::string& out_path)
{
  // Declare a lambda function for helper
  auto try_one = [&](const std::string& p) -> bool
  {
    if (!fs::exists(p)) return false;

    cv::FileStorage fsr(p, cv::FileStorage::READ);
    if (!fsr.isOpened()) return false;

    std::string serial;
    int width = 0, height = 0;

    fsr["serial"] >> serial;
    fsr["width"]  >> width;
    fsr["height"] >> height;

    if (serial.empty()) return false;

    out_serial = serial;
    out_width  = width;
    out_height = height;
    out_path   = p;
    return true;
  };

  // Prefer install mirrors first (runtime truth)
  for (const auto& d : targets.install_dirs)
    if (try_one(d + "/active_camera.yaml")) return true;

  // Then src mirrors
  for (const auto& d : targets.src_dirs)
    if (try_one(d + "/active_camera.yaml")) return true;

  return false;
}

class CameraExtrinsicsCalib : public rclcpp::Node
{
public:
  CameraExtrinsicsCalib() : Node("camera_extrinsics_calib")
  {
    camera_serial_ = this->declare_parameter<std::string>("camera_serial", "");
    tag_pose_topic_ = this->declare_parameter<std::string>("tag_pose_topic", "cMo");
    robot_pose_topic_ = this->declare_parameter<std::string>("robot_pose_topic", "/ur/tcp_pose");
    output_subdir_ = this->declare_parameter<std::string>("output_subdir", "config");
    output_packages_ = this->declare_parameter<std::vector<std::string>>(
      "output_packages", std::vector<std::string>{"uralignment_cpp","uralignment_py","calibration_cpp"});
    workspace_root_hint_ = this->declare_parameter<std::string>("workspace_root_hint", "");
    pass_mean_trans_m_ = this->declare_parameter<double>("pass_mean_trans_m", 0.002);
    pass_mean_rot_deg_ = this->declare_parameter<double>("pass_mean_rot_deg", 0.5);
    max_capture_dt_ms_ = this->declare_parameter<double>("max_capture_dt_ms", 30.0);
    robot_buffer_size_ = static_cast<size_t>(this->declare_parameter<int>("robot_buffer_size", 300));
    max_pair_dt_ms_ = this->declare_parameter<double>("max_pair_dt_ms", 15.0);
    fresh_tag_timeout_ms_ = this->declare_parameter<int>("fresh_tag_timeout_ms", 1000);

    targets_ = calibration_cpp::compute_write_targets(output_packages_, output_subdir_, workspace_root_hint_);

    // Auto-fill camera_serial from active_camera.yaml 
    if (camera_serial_.empty())
    {
      std::string ser, path;
      int w = 0, h = 0;

      if (read_active_camera_yaml_from_targets(targets_, ser, w, h, path))
      {
        camera_serial_ = ser;
        this->set_parameter(rclcpp::Parameter("camera_serial", camera_serial_));
        RCLCPP_INFO(this->get_logger(),
          "camera_serial not provided -> using serial=%s from %s (width=%d height=%d)",
          camera_serial_.c_str(), path.c_str(), w, h);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(),
          "camera_serial not provided and active_camera.yaml was not found/readable in install/src mirrors.");
      }
    }

    // Subscriptions:
    robot_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CameraExtrinsicsCalib::robot_cb, this, std::placeholders::_1));
    tag_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      tag_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CameraExtrinsicsCalib::tag_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "camera_extrinsics_calib ready. robot_pose_topic=%s tag_pose_topic=%s camera_serial=%s",
      robot_pose_topic_.c_str(), tag_pose_topic_.c_str(), camera_serial_.c_str());

  }

  void run_interactive()
  {
    calibration_cpp::TerminalRawMode raw;
    if (!raw.ok()) {
      RCLCPP_ERROR(this->get_logger(), "TerminalRawMode failed. Run from a real terminal.");
      return;
    }

    std::cout << "\n ########## camera_extrinsics_calib (solve eMc -> save ePc)  ##########\n";
    calibration_cpp::print_key_help();
    std::cout << "Samples captured: 0\n";

    while (rclcpp::ok())
    {
      const int k = calibration_cpp::read_key_nonblocking();
      if (k == calibration_cpp::KEY_NONE) { std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue; }

      if (k == 'q' || k == calibration_cpp::KEY_ESC) {
        std::cout << "\nCanceled. No calibration saved.\n";
        return;
      }

      if (k == 'c') {
        capture_sample();
        continue;
      }

      if (k == 'x' || k == 'X') {
        compute_and_save();
        return;
      }
    }
  }

private:
  std::string camera_serial_;
  std::string tag_pose_topic_;
  std::string robot_pose_topic_;
  std::string output_subdir_;
  std::vector<std::string> output_packages_;
  std::string workspace_root_hint_;

  double pass_mean_trans_m_{0.002};
  double pass_mean_rot_deg_{0.5};
  double max_capture_dt_ms_{30.0};

  calibration_cpp::WriteTargets targets_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tag_sub_;

  std::mutex m_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_robot_;
  geometry_msgs::msg::TransformStamped::SharedPtr last_tag_;

  std::vector<Eigen::Matrix4d> iMe_samples_;
  std::vector<Eigen::Matrix4d> cMo_samples_;

  std::condition_variable tag_cv_;
  uint64_t tag_seq_{0};

  std::deque<geometry_msgs::msg::PoseStamped> robot_buf_;
  size_t robot_buffer_size_{300};

  double max_pair_dt_ms_{15.0};
  int fresh_tag_timeout_ms_{1000};

  // Store latest robot pose
  void robot_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(m_);
    robot_buf_.push_back(*msg);
    while (robot_buf_.size() > robot_buffer_size_) {
      robot_buf_.pop_front();
    }
    last_robot_ = msg;
  }

  // store latest tag pose
  void tag_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(m_);
    last_tag_ = msg;
    ++tag_seq_;
    tag_cv_.notify_all();
  }

  // Wait until a tag sample newer than a recorded sequence number arrives
  bool wait_for_fresh_tag_after(
    uint64_t seq_before,
    geometry_msgs::msg::TransformStamped::SharedPtr& out_tag,
    std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lk(m_);
    const bool ok = tag_cv_.wait_for(
      lk, timeout,
      [&]() { return tag_seq_ > seq_before && last_tag_ != nullptr; });

    if (!ok) return false;
    out_tag = last_tag_;
    return true;
  }

  // Fine the robot pose with the timestamp closest to the target time (from tag sample)
  bool find_nearest_robot_locked(
    const builtin_interfaces::msg::Time& target_stamp,
    geometry_msgs::msg::PoseStamped& out_robot,
    double& out_dt_ms) const
  {
    if (robot_buf_.empty()) return false;

    const int64_t target_ns = rclcpp::Time(target_stamp).nanoseconds();
    int64_t best_abs_ns = std::numeric_limits<int64_t>::max();
    const geometry_msgs::msg::PoseStamped* best = nullptr;

    for (const auto& r : robot_buf_)
    {
      const int64_t dt_ns =
        std::llabs(rclcpp::Time(r.header.stamp).nanoseconds() - target_ns);

      if (dt_ns < best_abs_ns)
      {
        best_abs_ns = dt_ns;
        best = &r;
      }
    }

    if (!best) return false;

    out_robot = *best;
    out_dt_ms = static_cast<double>(best_abs_ns) / 1e6;
    return true;
  }

  // Capture one paired robot and vision sample for extrinsic calibration
  void capture_sample()
  {
    uint64_t seq_before = 0;
    {
      std::lock_guard<std::mutex> lk(m_);
      seq_before = tag_seq_;
    }

    geometry_msgs::msg::TransformStamped::SharedPtr t;
    if (!wait_for_fresh_tag_after(
          seq_before,
          t,
          std::chrono::milliseconds(fresh_tag_timeout_ms_)))
    {
      std::cout << "Capture ignored: timed out waiting for fresh cMo after keypress.\n";
      return;
    }

    geometry_msgs::msg::PoseStamped r_match;
    double best_dt_ms = 0.0;
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!find_nearest_robot_locked(t->header.stamp, r_match, best_dt_ms))
      {
        std::cout << "Capture ignored: no RTDE samples in buffer.\n";
        return;
      }
    }

    if (best_dt_ms > max_pair_dt_ms_)
    {
      std::cout << "Capture ignored: nearest RTDE sample too far from cMo ("
                << best_dt_ms << " ms > " << max_pair_dt_ms_ << " ms)\n";
      return;
    }

    iMe_samples_.push_back(calibration_cpp::pose_stamped_to_T(r_match));
    cMo_samples_.push_back(calibration_cpp::tf_stamped_to_T(*t));

    std::cout << "Captured sample #" << iMe_samples_.size()
              << " (best |dt|=" << best_dt_ms << " ms)\n";
  }

  // Solve and report residuals
  void compute_and_save()
  {
    if (camera_serial_.empty()) {
    std::cout << "NOTE: camera_serial is empty. Saving generic ePc.yaml.\n";
    }
    

    if (iMe_samples_.size() < 3 || cMo_samples_.size() < 3) {
      std::cout << "Need >= 3 samples. Have " << iMe_samples_.size() << "\n";
      return;
    }

    // Build A and B for A X = X B using consecutive pairs
    std::vector<Eigen::Matrix4d> A, B;
    for (size_t i=0;i+1<iMe_samples_.size();++i)
    {
      Eigen::Matrix4d A_i = calibration_cpp::T_inv(iMe_samples_[i]) * iMe_samples_[i+1];
      Eigen::Matrix4d B_i = cMo_samples_[i] * calibration_cpp::T_inv(cMo_samples_[i+1]);

      A.push_back(A_i);
      B.push_back(B_i);
    }

    Eigen::Matrix4d X = calibration_cpp::solve_AX_XB(A, B); // X = eMc

    double mean_t, max_t, mean_r, max_r;
    calibration_cpp::compute_AX_XB_residuals(A, B, X, mean_t, max_t, mean_r, max_r);

    const double mean_deg = mean_r * 180.0 / M_PI;
    const bool pass = (mean_t <= pass_mean_trans_m_) && (mean_deg <= pass_mean_rot_deg_);

    std::cout << "\nSolved eMc.\n";
    std::cout << "Residuals: mean_trans=" << mean_t << " m, max_trans=" << max_t
              << " m, mean_rot=" << mean_deg << " deg, max_rot=" << (max_r*180.0/M_PI) << " deg\n";
    std::cout << "PASS flag: " << (pass ? "PASS" : "FAIL (flag only)") << "\n";
    const auto pose6 = calibration_cpp::T_to_pose6(X);
    const std::string fname = "ePc.yaml";
    calibration_cpp::write_pose6_yaml_multi(targets_, fname, pose6);

    std::cout << "Saved " << fname << " to install + src mirrors for packages: ";
    for (auto& p : output_packages_) std::cout << p << " ";
    std::cout << "\n";
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraExtrinsicsCalib>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thr([&](){ exec.spin(); });

  node->run_interactive();

  exec.cancel();
  if (spin_thr.joinable()) spin_thr.join();
  rclcpp::shutdown();
  return 0;
}