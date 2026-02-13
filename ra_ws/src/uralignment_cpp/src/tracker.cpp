#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;

struct SlidingRate1s
{
  std::deque<SteadyClock::time_point> t;

  void push(const SteadyClock::time_point & now)
  {
    t.push_back(now);
    prune(now);
  }

  double rate(const SteadyClock::time_point & now)
  {
    prune(now);
    return static_cast<double>(t.size()); // 1s window => Hz == count
  }

private:
  void prune(const SteadyClock::time_point & now)
  {
    const auto cutoff = now - std::chrono::seconds(1);
    while (!t.empty() && t.front() < cutoff) {
      t.pop_front();
    }
  }
};

static std::string timestamp_utc_compact()
{
  std::time_t tt = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &tt);
#else
  gmtime_r(&tt, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

static void ensure_dir(const fs::path & p)
{
  std::error_code ec;
  fs::create_directories(p, ec);
  if (ec) {
    throw std::runtime_error("Failed to create directory: " + p.string() + " : " + ec.message());
  }
}

static void write_cpo_yaml_6x1(const fs::path & yaml_path, const std::array<double, 6> & cpo)
{
  std::ofstream out(yaml_path, std::ios::out | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("Failed to open YAML for writing: " + yaml_path.string());
  }

  out << "rows: 6\n";
  out << "cols: 1\n";
  out << "data:\n";
  for (int i = 0; i < 6; ++i) {
    out << "  - [" << std::setprecision(15) << cpo[i] << "]\n";
  }
  out.flush();
}

// Convert quaternion to theta-u vector (axis-angle with theta folded into axis components).
// Returns (theta*ux, theta*uy, theta*uz). This matches the common ViSP "ThetaU" vector representation.
static std::array<double, 3> quat_to_theta_u(double qx, double qy, double qz, double qw)
{
  // Normalize
  const double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if (n > 1e-12) { qx/=n; qy/=n; qz/=n; qw/=n; }

  // Flip sign to prefer shortest rotation
  if (qw < 0.0) { qw=-qw; qx=-qx; qy=-qy; qz=-qz; }

  const double qw_clamped = std::clamp(qw, -1.0, 1.0);
  const double theta = 2.0 * std::acos(qw_clamped);

  // s = |sin(theta/2)|
  const double s = std::sqrt(std::max(0.0, 1.0 - qw_clamped*qw_clamped));

  if (s < 1e-9 || theta < 1e-9) {
    return {0.0, 0.0, 0.0};
  }

  const double ux = qx / s;
  const double uy = qy / s;
  const double uz = qz / s;

  return {theta*ux, theta*uy, theta*uz};
}

static void plot_rates_csv_to_png(const fs::path & csv_path, const fs::path & png_path)
{
  std::ifstream in(csv_path);
  if (!in) {
    throw std::runtime_error("Failed to open CSV: " + csv_path.string());
  }

  std::string line;
  if (!std::getline(in, line)) {
    throw std::runtime_error("CSV is empty: " + csv_path.string());
  }

  std::vector<double> t, hz_img, hz_cmo, hz_vc;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string cell;

    std::vector<double> row;
    while (std::getline(ss, cell, ',')) {
      try { row.push_back(std::stod(cell)); }
      catch (...) { row.push_back(0.0); }
    }
    if (row.size() < 4) continue;

    t.push_back(row[0]);
    hz_img.push_back(row[1]);
    hz_cmo.push_back(row[2]);
    hz_vc.push_back(row[3]);
  }

  const int W = 1400, H = 800;
  const int L = 90, R = 30, T = 30, B = 80;
  cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

  if (t.empty()) {
    cv::putText(img, "No data in rates.csv", cv::Point(50, 100),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
    cv::imwrite(png_path.string(), img);
    return;
  }

  const double t_min = t.front();
  const double t_max = t.back();

  double hz_max = 0.0;
  for (size_t i = 0; i < t.size(); ++i) {
    hz_max = std::max(hz_max, hz_img[i]);
    hz_max = std::max(hz_max, hz_cmo[i]);
    hz_max = std::max(hz_max, hz_vc[i]);
  }
  if (hz_max < 1e-6) hz_max = 1.0;
  hz_max *= 1.10;

  const int x0 = L, y0 = H - B;
  const int x1 = W - R, y1 = T;

  cv::line(img, cv::Point(x0, y0), cv::Point(x1, y0), cv::Scalar(0, 0, 0), 2);
  cv::line(img, cv::Point(x0, y0), cv::Point(x0, y1), cv::Scalar(0, 0, 0), 2);

  cv::putText(img, "time [s]", cv::Point((x0 + x1) / 2 - 50, H - 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 0), 2);
  cv::putText(img, "rate [Hz] (msgs/sec over last 1s)", cv::Point(50, 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);

  auto to_x = [&](double tt) -> int {
    const double denom = (t_max - t_min);
    const double a = (denom > 1e-9) ? ((tt - t_min) / denom) : 0.0;
    return x0 + static_cast<int>(a * (x1 - x0));
  };

  auto to_y = [&](double hz) -> int {
    const double a = hz / hz_max;
    return y0 - static_cast<int>(a * (y0 - y1));
  };

  // Ticks
  {
    const int n_xticks = 10;
    for (int i = 0; i <= n_xticks; ++i) {
      const double tt = t_min + (t_max - t_min) * (static_cast<double>(i) / n_xticks);
      const int x = to_x(tt);
      cv::line(img, cv::Point(x, y0), cv::Point(x, y0 + 8), cv::Scalar(0, 0, 0), 1);
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1) << tt;
      cv::putText(img, oss.str(), cv::Point(x - 20, y0 + 35),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    const int n_yticks = 8;
    for (int i = 0; i <= n_yticks; ++i) {
      const double hz = hz_max * (static_cast<double>(i) / n_yticks);
      const int y = to_y(hz);
      cv::line(img, cv::Point(x0 - 8, y), cv::Point(x0, y), cv::Scalar(0, 0, 0), 1);
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1) << hz;
      cv::putText(img, oss.str(), cv::Point(10, y + 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
  }

  auto draw_series = [&](const std::vector<double> & yv, const cv::Scalar & color) {
    std::vector<cv::Point> pts;
    pts.reserve(t.size());
    for (size_t i = 0; i < t.size(); ++i) {
      pts.emplace_back(to_x(t[i]), to_y(yv[i]));
    }
    if (pts.size() >= 2) {
      cv::polylines(img, pts, false, color, 2, cv::LINE_AA);
    }
  };

  // Lines (BGR)
  draw_series(hz_img, cv::Scalar(0, 0, 255));   // red
  draw_series(hz_cmo, cv::Scalar(0, 160, 0));   // green
  draw_series(hz_vc,  cv::Scalar(255, 0, 0));   // blue

  // Legend
  {
    const int lx = W - 380;
    const int ly = 60;
    cv::rectangle(img, cv::Rect(lx, ly, 350, 110), cv::Scalar(245, 245, 245), cv::FILLED);
    cv::rectangle(img, cv::Rect(lx, ly, 350, 110), cv::Scalar(0, 0, 0), 1);

    cv::line(img, cv::Point(lx + 20, ly + 30), cv::Point(lx + 80, ly + 30), cv::Scalar(0, 0, 255), 3);
    cv::putText(img, "/camera/color/image_raw", cv::Point(lx + 95, ly + 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

    cv::line(img, cv::Point(lx + 20, ly + 60), cv::Point(lx + 80, ly + 60), cv::Scalar(0, 160, 0), 3);
    cv::putText(img, "/cMo", cv::Point(lx + 95, ly + 66),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

    cv::line(img, cv::Point(lx + 20, ly + 90), cv::Point(lx + 80, ly + 90), cv::Scalar(255, 0, 0), 3);
    cv::putText(img, "/v_c", cv::Point(lx + 95, ly + 96),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
  }

  cv::imwrite(png_path.string(), img);
}

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("tracker", options)
  {
    // Parameters
    data_dir_param_ = this->declare_parameter<std::string>("data_dir", "");
    log_hz_ = this->declare_parameter<double>("log_hz", 10.0);
    if (log_hz_ < 1.0) log_hz_ = 1.0;

    // Topics (keep explicit so you can remap later if you ever want)
    topic_image_ = this->declare_parameter<std::string>("topic_image", "/camera/color/image_raw");
    topic_cmo_   = this->declare_parameter<std::string>("topic_cMo",   "/cMo");
    topic_vc_    = this->declare_parameter<std::string>("topic_v_c",   "/v_c");

    // Resolve data directory
    base_data_dir_ = resolve_data_dir();
    ensure_dir(base_data_dir_);

    runs_dir_ = base_data_dir_ / "tracker_runs";
    ensure_dir(runs_dir_);

    run_dir_ = runs_dir_ / ("run_" + timestamp_utc_compact());
    ensure_dir(run_dir_);

    rates_csv_path_ = run_dir_ / "rates.csv";
    rates_png_path_ = run_dir_ / "rates.png";

    // This is the file you want overwritten continuously in your chosen data folder.
    cpo_yaml_path_ = base_data_dir_ / "ur_pose_cPo.yaml";

    // Open CSV
    csv_.open(rates_csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_) {
      throw std::runtime_error("Failed to open rates CSV: " + rates_csv_path_.string());
    }
    csv_ << "t_sec,hz_image_raw,hz_cMo,hz_v_c\n";
    csv_.flush();

    start_ = SteadyClock::now();

    RCLCPP_INFO(this->get_logger(), "tracker started");
    RCLCPP_INFO(this->get_logger(), "Data dir:      %s", base_data_dir_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "Run dir:       %s", run_dir_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "rates.csv:     %s", rates_csv_path_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "rates.png:     %s (written on shutdown)", rates_png_path_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "ur_pose_cPo:   %s (rewritten on every /cMo message)", cpo_yaml_path_.string().c_str());

    // /camera/color/image_raw subscription (known type)
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_image_, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr) {
          on_msg("image_raw");
        });
    }

    // /v_c is often Float64MultiArray; we still detect at runtime (subscribe when it appears)
    // /cMo is TransformStamped in your system; we detect and then subscribe as TransformStamped.
    sub_probe_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      [this]() { this->try_create_dynamic_subscriptions(); });

    // Logging timer
    log_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / log_hz_)),
      [this]() { this->log_once(); });
  }

  ~TrackerNode() override
  {
    on_shutdown_plot_and_close();
  }

private:
  fs::path resolve_data_dir()
  {
    if (!data_dir_param_.empty()) {
      return fs::path(data_dir_param_);
    }

    // Prefer source-tree data dir when running from ra_ws
    // Example: /home/giffomatic/Robot-Alignment/ra_ws
    // candidate: /home/giffomatic/Robot-Alignment/ra_ws/src/uralignment_cpp/data
    fs::path candidate = fs::current_path() / "src" / "uralignment_cpp" / "data";
    if (fs::exists(candidate) && fs::is_directory(candidate)) {
      return candidate;
    }

    // Fallback to installed share directory
    const std::string pkg_name = "uralignment_cpp"; // MUST match package.xml <name>
    return fs::path(ament_index_cpp::get_package_share_directory(pkg_name)) / "data";
  }

  void on_msg(const std::string & key)
  {
    const auto now = SteadyClock::now();
    std::scoped_lock lk(mutex_);
    rates_[key].push(now);
  }

  void log_once()
  {
    const auto now = SteadyClock::now();
    const double t_sec = std::chrono::duration<double>(now - start_).count();

    double hz_img = 0.0, hz_cmo = 0.0, hz_vc = 0.0;
    {
      std::scoped_lock lk(mutex_);
      hz_img = rates_["image_raw"].rate(now);
      hz_cmo = rates_["cMo"].rate(now);
      hz_vc  = rates_["v_c"].rate(now);
    }

    csv_ << std::fixed << std::setprecision(6)
         << t_sec << ","
         << hz_img << ","
         << hz_cmo << ","
         << hz_vc << "\n";
    csv_.flush();
  }

  void try_create_dynamic_subscriptions()
  {
    if (sub_cmo_ && sub_vc_) {
      sub_probe_timer_->cancel();
      return;
    }

    const auto topics = this->get_topic_names_and_types();

    auto topic_type = [&](const std::string & topic) -> std::optional<std::string> {
      auto it = topics.find(topic);
      if (it == topics.end() || it->second.empty()) return std::nullopt;
      return it->second.front();
    };

    // /cMo: TransformStamped (rate + YAML overwrite each message)
    if (!sub_cmo_) {
      const auto t = topic_type(topic_cmo_);
      if (t && *t == "geometry_msgs/msg/TransformStamped") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        sub_cmo_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
          topic_cmo_, qos,
          [this](geometry_msgs::msg::TransformStamped::ConstSharedPtr msg) {
            on_msg("cMo");

            const auto & tr = msg->transform.translation;
            const auto & q  = msg->transform.rotation;

            const auto tu = quat_to_theta_u(q.x, q.y, q.z, q.w);
            const std::array<double, 6> v = {tr.x, tr.y, tr.z, tu[0], tu[1], tu[2]};

            // Overwrite ur_pose_cPo.yaml EVERY message
            try {
              write_cpo_yaml_6x1(cpo_yaml_path_, v);
            } catch (const std::exception & e) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "tracker: failed writing ur_pose_cPo.yaml: %s", e.what());
            }
          });

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s as geometry_msgs/msg/TransformStamped (rate + YAML)",
                    topic_cmo_.c_str());
      } else if (t && *t != "geometry_msgs/msg/TransformStamped") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tracker: topic %s has unsupported type '%s' for cMo (expected TransformStamped)",
                             topic_cmo_.c_str(), t->c_str());
      }
    }

    // /v_c: support Float64MultiArray (counting only)
    if (!sub_vc_) {
      const auto t = topic_type(topic_vc_);
      if (t && *t == "std_msgs/msg/Float64MultiArray") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        sub_vc_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          topic_vc_, qos,
          [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr) {
            on_msg("v_c");
          });
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s as std_msgs/msg/Float64MultiArray", topic_vc_.c_str());
      } else if (t && *t != "std_msgs/msg/Float64MultiArray") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tracker: topic %s has unsupported type '%s' for v_c (expected Float64MultiArray)",
                             topic_vc_.c_str(), t->c_str());
      }
    }
  }

  void on_shutdown_plot_and_close()
  {
    if (shutdown_done_) return;
    shutdown_done_ = true;

    try {
      if (csv_.is_open()) {
        csv_.flush();
        csv_.close();
      }
      plot_rates_csv_to_png(rates_csv_path_, rates_png_path_);
    } catch (...) {
      // no-throw on shutdown
    }
  }

private:
  // Params
  std::string data_dir_param_;
  double log_hz_{10.0};

  std::string topic_image_;
  std::string topic_cmo_;
  std::string topic_vc_;

  // Paths
  fs::path base_data_dir_;
  fs::path runs_dir_;
  fs::path run_dir_;
  fs::path rates_csv_path_;
  fs::path rates_png_path_;
  fs::path cpo_yaml_path_;

  // Logging
  std::ofstream csv_;
  SteadyClock::time_point start_;

  // Rate tracking
  std::mutex mutex_;
  std::unordered_map<std::string, SlidingRate1s> rates_;

  // Subs/Timers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::SubscriptionBase::SharedPtr sub_cmo_;
  rclcpp::SubscriptionBase::SharedPtr sub_vc_;

  rclcpp::TimerBase::SharedPtr sub_probe_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  bool shutdown_done_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}