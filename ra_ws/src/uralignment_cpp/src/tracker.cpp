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

struct Sample6
{
  double t_sec{0.0};
  std::array<double, 6> v{};
};

struct Sample2
{
  double t_sec{0.0};
  std::array<double, 2> v{};
};

static constexpr double kRad2Deg = 57.2957795130823208768;

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

static std::array<double, 3> quat_to_theta_u(double qx, double qy, double qz, double qw)
{
  const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n > 1e-12) {
    qx /= n;
    qy /= n;
    qz /= n;
    qw /= n;
  }

  if (qw < 0.0) {
    qw = -qw;
    qx = -qx;
    qy = -qy;
    qz = -qz;
  }

  const double qw_clamped = std::clamp(qw, -1.0, 1.0);
  const double theta = 2.0 * std::acos(qw_clamped);
  const double s = std::sqrt(std::max(0.0, 1.0 - qw_clamped * qw_clamped));

  if (s < 1e-9 || theta < 1e-9) {
    return {0.0, 0.0, 0.0};
  }

  const double ux = qx / s;
  const double uy = qy / s;
  const double uz = qz / s;

  return {theta * ux, theta * uy, theta * uz};
}

static std::array<double, 2> compute_error_norms_from_errors_xyz(const std::array<double, 6> & e)
{
  const double tr_norm =
    std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);

  const double tu_norm_deg =
    std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]) * kRad2Deg;

  return {tr_norm, tu_norm_deg};
}

static cv::Mat make_timeseries_panel(
  const std::string & title,
  const std::string & y_units,
  const std::vector<double> & t,
  const std::vector<std::vector<double>> & series,
  const std::vector<std::string> & labels,
  const std::vector<cv::Scalar> & colors)
{
  const int W = 900;
  const int H = 520;
  const int L = 85;
  const int R = 20;
  const int T = 55;
  const int B = 65;

  cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

  cv::putText(img, title, cv::Point(18, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
  cv::putText(img, "time [s]", cv::Point((W - L - R) / 2 + 30, H - 18),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
  cv::putText(img, y_units, cv::Point(18, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1);

  if (t.empty() || series.empty()) {
    cv::putText(img, "No data", cv::Point(120, H / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
    return img;
  }

  bool have_y = false;
  double y_min = 0.0;
  double y_max = 0.0;

  for (const auto & s : series) {
    for (double y : s) {
      if (!std::isfinite(y)) {
        continue;
      }
      if (!have_y) {
        y_min = y;
        y_max = y;
        have_y = true;
      } else {
        y_min = std::min(y_min, y);
        y_max = std::max(y_max, y);
      }
    }
  }

  if (!have_y) {
    cv::putText(img, "No finite data", cv::Point(120, H / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
    return img;
  }

  const double t_min = t.front();
  const double t_max = t.back();

  if (std::abs(y_max - y_min) < 1e-12) {
    const double pad = std::max(1.0, std::abs(y_max) * 0.10 + 1e-3);
    y_min -= pad;
    y_max += pad;
  } else {
    const double pad = 0.10 * (y_max - y_min);
    y_min -= pad;
    y_max += pad;
  }

  const int x0 = L;
  const int y0 = H - B;
  const int x1 = W - R;
  const int y1 = T;

  cv::rectangle(img, cv::Rect(x0, y1, x1 - x0, y0 - y1), cv::Scalar(255, 255, 255), cv::FILLED);
  cv::rectangle(img, cv::Rect(x0, y1, x1 - x0, y0 - y1), cv::Scalar(0, 0, 0), 1);

  const double t_span = std::max(1e-9, t_max - t_min);
  const double y_span = std::max(1e-9, y_max - y_min);

  auto to_x = [&](double tt) -> int {
    const double a = (tt - t_min) / t_span;
    return x0 + static_cast<int>(a * (x1 - x0));
  };

  auto to_y = [&](double yy) -> int {
    const double a = (yy - y_min) / y_span;
    return y0 - static_cast<int>(a * (y0 - y1));
  };

  const int n_xticks = 6;
  for (int i = 0; i <= n_xticks; ++i) {
    const double tt = t_min + (t_max - t_min) * (static_cast<double>(i) / n_xticks);
    const int x = to_x(tt);

    cv::line(img, cv::Point(x, y0), cv::Point(x, y1), cv::Scalar(230, 230, 230), 1);
    cv::line(img, cv::Point(x, y0), cv::Point(x, y0 + 6), cv::Scalar(0, 0, 0), 1);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << tt;
    cv::putText(img, oss.str(), cv::Point(x - 20, y0 + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1);
  }

  const int n_yticks = 6;
  for (int i = 0; i <= n_yticks; ++i) {
    const double yy = y_min + (y_max - y_min) * (static_cast<double>(i) / n_yticks);
    const int y = to_y(yy);

    cv::line(img, cv::Point(x0, y), cv::Point(x1, y), cv::Scalar(230, 230, 230), 1);
    cv::line(img, cv::Point(x0 - 6, y), cv::Point(x0, y), cv::Scalar(0, 0, 0), 1);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << yy;
    cv::putText(img, oss.str(), cv::Point(5, y + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 0, 0), 1);
  }

  for (size_t sidx = 0; sidx < series.size(); ++sidx) {
    const auto & s = series[sidx];
    const size_t n = std::min(t.size(), s.size());
    if (n < 2) {
      continue;
    }

    std::vector<cv::Point> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      pts.emplace_back(to_x(t[i]), to_y(s[i]));
    }

    const cv::Scalar color = colors[sidx % colors.size()];
    cv::polylines(img, pts, false, color, 2, cv::LINE_AA);
  }

  if (!labels.empty()) {
    const int leg_w = 180;
    const int leg_h = 26 * static_cast<int>(labels.size()) + 12;
    const int lx = x1 - leg_w - 10;
    const int ly = y1 + 10;

    cv::rectangle(img, cv::Rect(lx, ly, leg_w, leg_h), cv::Scalar(245, 245, 245), cv::FILLED);
    cv::rectangle(img, cv::Rect(lx, ly, leg_w, leg_h), cv::Scalar(0, 0, 0), 1);

    for (size_t i = 0; i < labels.size(); ++i) {
      const int yy = ly + 22 + static_cast<int>(i) * 26;
      const cv::Scalar color = colors[i % colors.size()];
      cv::line(img, cv::Point(lx + 12, yy - 5), cv::Point(lx + 42, yy - 5), color, 3);
      cv::putText(img, labels[i], cv::Point(lx + 52, yy),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
  }

  return img;
}

static void plot_tracking_summary_to_png(
  const fs::path & png_path,
  const std::vector<Sample6> & vc_samples,
  const std::vector<Sample6> & error_xyz_samples,
  const std::vector<Sample2> & error_norm_samples)
{
  std::vector<double> t_vc;
  std::vector<std::vector<double>> vc_linear(3), vc_angular(3);

  for (const auto & s : vc_samples) {
    t_vc.push_back(s.t_sec);
    vc_linear[0].push_back(s.v[0]);
    vc_linear[1].push_back(s.v[1]);
    vc_linear[2].push_back(s.v[2]);
    vc_angular[0].push_back(s.v[3]);
    vc_angular[1].push_back(s.v[4]);
    vc_angular[2].push_back(s.v[5]);
  }

  const size_t n_err = std::min(error_xyz_samples.size(), error_norm_samples.size());

  std::vector<double> t_err;
  std::vector<std::vector<double>> err_translation(4), err_rotation(4);

  for (size_t i = 0; i < n_err; ++i) {
    const auto & e = error_xyz_samples[i].v;
    const auto & n = error_norm_samples[i].v;

    t_err.push_back(error_xyz_samples[i].t_sec);

    err_translation[0].push_back(e[0]);
    err_translation[1].push_back(e[1]);
    err_translation[2].push_back(e[2]);
    err_translation[3].push_back(n[0]);

    err_rotation[0].push_back(e[3] * kRad2Deg);
    err_rotation[1].push_back(e[4] * kRad2Deg);
    err_rotation[2].push_back(e[5] * kRad2Deg);
    err_rotation[3].push_back(n[1]);
  }

  const std::vector<cv::Scalar> colors3 = {
    cv::Scalar(0, 0, 255),
    cv::Scalar(0, 160, 0),
    cv::Scalar(255, 0, 0)
  };

  const std::vector<cv::Scalar> colors4 = {
    cv::Scalar(0, 0, 255),
    cv::Scalar(0, 160, 0),
    cv::Scalar(255, 0, 0),
    cv::Scalar(0, 0, 0)
  };

  const cv::Mat p1 = make_timeseries_panel(
    "Camera-frame linear velocity", "m/s",
    t_vc, vc_linear,
    {"vx", "vy", "vz"}, colors3);

  const cv::Mat p2 = make_timeseries_panel(
    "Camera-frame angular velocity", "rad/s",
    t_vc, vc_angular,
    {"wx", "wy", "wz"}, colors3);

  const cv::Mat p3 = make_timeseries_panel(
    "Translation error + norm", "m",
    t_err, err_translation,
    {"ex", "ey", "ez", "||e_t||"}, colors4);

  const cv::Mat p4 = make_timeseries_panel(
    "Rotation error + norm", "deg",
    t_err, err_rotation,
    {"tu_x", "tu_y", "tu_z", "||e_tu||"}, colors4);

  cv::Mat canvas(p1.rows * 2, p1.cols * 2, CV_8UC3, cv::Scalar(255, 255, 255));

  p1.copyTo(canvas(cv::Rect(0, 0, p1.cols, p1.rows)));
  p2.copyTo(canvas(cv::Rect(p1.cols, 0, p2.cols, p2.rows)));
  p3.copyTo(canvas(cv::Rect(0, p1.rows, p3.cols, p3.rows)));
  p4.copyTo(canvas(cv::Rect(p1.cols, p1.rows, p4.cols, p4.rows)));

  cv::line(canvas, cv::Point(p1.cols, 0), cv::Point(p1.cols, canvas.rows), cv::Scalar(0, 0, 0), 1);
  cv::line(canvas, cv::Point(0, p1.rows), cv::Point(canvas.cols, p1.rows), cv::Scalar(0, 0, 0), 1);

  cv::imwrite(png_path.string(), canvas);
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
      try {
        row.push_back(std::stod(cell));
      } catch (...) {
        row.push_back(0.0);
      }
    }
    if (row.size() < 4) continue;

    t.push_back(row[0]);
    hz_img.push_back(row[1]);
    hz_cmo.push_back(row[2]);
    hz_vc.push_back(row[3]);
  }

  const int W = 1400;
  const int H = 800;
  const int L = 90;
  const int R = 30;
  const int T = 30;
  const int B = 80;
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

  const int x0 = L;
  const int y0 = H - B;
  const int x1 = W - R;
  const int y1 = T;

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

  draw_series(hz_img, cv::Scalar(0, 0, 255));
  draw_series(hz_cmo, cv::Scalar(0, 160, 0));
  draw_series(hz_vc, cv::Scalar(255, 0, 0));

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
    data_dir_param_ = this->declare_parameter<std::string>("data_dir", "");
    log_hz_ = this->declare_parameter<double>("log_hz", 10.0);
    if (log_hz_ < 1.0) {
      log_hz_ = 1.0;
    }

    topic_image_      = this->declare_parameter<std::string>("topic_image", "/camera/color/image_raw");
    topic_cmo_        = this->declare_parameter<std::string>("topic_cMo", "/cMo");
    topic_vc_         = this->declare_parameter<std::string>("topic_v_c", "/v_c");
    topic_errors_xyz_ = this->declare_parameter<std::string>("topic_errors_xyz", "/errors_xyz");

    base_data_dir_ = resolve_data_dir();
    ensure_dir(base_data_dir_);

    runs_dir_ = base_data_dir_ / "tracker_runs";
    ensure_dir(runs_dir_);

    run_dir_ = runs_dir_ / ("run_" + timestamp_utc_compact());
    ensure_dir(run_dir_);

    rates_csv_path_    = run_dir_ / "rates.csv";
    rates_png_path_    = run_dir_ / "rates.png";
    tracking_png_path_ = run_dir_ / "tracking_summary.png";

    cpo_yaml_path_ = base_data_dir_ / "ur_pose_cPo.yaml";

    csv_.open(rates_csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_) {
      throw std::runtime_error("Failed to open rates CSV: " + rates_csv_path_.string());
    }
    csv_ << "t_sec,hz_image_raw,hz_cMo,hz_v_c\n";
    csv_.flush();

    start_ = SteadyClock::now();

    RCLCPP_INFO(this->get_logger(), "tracker started");
    RCLCPP_INFO(this->get_logger(), "Data dir:              %s", base_data_dir_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "Run dir:               %s", run_dir_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "rates.csv:             %s", rates_csv_path_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "rates.png:             %s (written on shutdown)", rates_png_path_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "tracking_summary.png:  %s (written on shutdown)", tracking_png_path_.string().c_str());
    RCLCPP_INFO(this->get_logger(), "ur_pose_cPo:           %s (rewritten on every /cMo message)", cpo_yaml_path_.string().c_str());

    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_image_, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr) {
          on_msg("image_raw");
        });
    }

    sub_probe_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      [this]() { this->try_create_dynamic_subscriptions(); });

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

    fs::path candidate = fs::current_path() / "src" / "uralignment_cpp" / "data";
    if (fs::exists(candidate) && fs::is_directory(candidate)) {
      return candidate;
    }

    const std::string pkg_name = "uralignment_cpp";
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

    double hz_img = 0.0;
    double hz_cmo = 0.0;
    double hz_vc = 0.0;
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
    if (sub_cmo_ && sub_vc_ && sub_errors_xyz_) {
      sub_probe_timer_->cancel();
      return;
    }

    const auto topics = this->get_topic_names_and_types();

    auto topic_type = [&](const std::string & topic) -> std::optional<std::string> {
      auto it = topics.find(topic);
      if (it == topics.end() || it->second.empty()) {
        return std::nullopt;
      }
      return it->second.front();
    };

    if (!sub_cmo_) {
      const auto t = topic_type(topic_cmo_);
      if (t && *t == "geometry_msgs/msg/TransformStamped") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        sub_cmo_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
          topic_cmo_, qos,
          [this](geometry_msgs::msg::TransformStamped::ConstSharedPtr msg) {
            if (!msg) {
              return;
            }

            on_msg("cMo");

            const auto & tr = msg->transform.translation;
            const auto & q  = msg->transform.rotation;

            const auto tu = quat_to_theta_u(q.x, q.y, q.z, q.w);
            const std::array<double, 6> v = {tr.x, tr.y, tr.z, tu[0], tu[1], tu[2]};

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

    if (!sub_vc_) {
      const auto t = topic_type(topic_vc_);
      if (t && *t == "std_msgs/msg/Float64MultiArray") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        sub_vc_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          topic_vc_, qos,
          [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
            if (!msg) {
              return;
            }

            const auto now = SteadyClock::now();
            const double t_sec = std::chrono::duration<double>(now - start_).count();

            std::array<double, 6> v{};
            const size_t n = std::min<size_t>(6, msg->data.size());
            for (size_t i = 0; i < n; ++i) {
              v[i] = msg->data[i];
            }

            std::scoped_lock lk(mutex_);
            rates_["v_c"].push(now);

            Sample6 s;
            s.t_sec = t_sec;
            s.v = v;
            vc_samples_.push_back(s);
          });

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s as std_msgs/msg/Float64MultiArray (6D velocity history)",
                    topic_vc_.c_str());
      } else if (t && *t != "std_msgs/msg/Float64MultiArray") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tracker: topic %s has unsupported type '%s' for v_c (expected Float64MultiArray)",
                             topic_vc_.c_str(), t->c_str());
      }
    }

    if (!sub_errors_xyz_) {
      const auto t = topic_type(topic_errors_xyz_);
      if (t && *t == "std_msgs/msg/Float64MultiArray") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        sub_errors_xyz_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          topic_errors_xyz_, qos,
          [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
            if (!msg) {
              return;
            }

            const auto now = SteadyClock::now();
            const double t_sec = std::chrono::duration<double>(now - start_).count();

            std::array<double, 6> e{};
            const size_t n = std::min<size_t>(6, msg->data.size());
            for (size_t i = 0; i < n; ++i) {
              e[i] = msg->data[i];
            }

            const auto norms = compute_error_norms_from_errors_xyz(e);

            std::scoped_lock lk(mutex_);

            Sample6 s6;
            s6.t_sec = t_sec;
            s6.v = e;
            error_xyz_samples_.push_back(s6);

            Sample2 s2;
            s2.t_sec = t_sec;
            s2.v = norms;
            error_norm_samples_.push_back(s2);
          });

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s as std_msgs/msg/Float64MultiArray (6D error history + local norms)",
                    topic_errors_xyz_.c_str());
      } else if (t && *t != "std_msgs/msg/Float64MultiArray") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tracker: topic %s has unsupported type '%s' for errors_xyz (expected Float64MultiArray)",
                             topic_errors_xyz_.c_str(), t->c_str());
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

      std::vector<Sample6> vc_samples_copy;
      std::vector<Sample6> error_xyz_samples_copy;
      std::vector<Sample2> error_norm_samples_copy;

      {
        std::scoped_lock lk(mutex_);
        vc_samples_copy = vc_samples_;
        error_xyz_samples_copy = error_xyz_samples_;
        error_norm_samples_copy = error_norm_samples_;
      }

      plot_rates_csv_to_png(rates_csv_path_, rates_png_path_);
      plot_tracking_summary_to_png(
        tracking_png_path_,
        vc_samples_copy,
        error_xyz_samples_copy,
        error_norm_samples_copy);
    } catch (...) {
      // no-throw on shutdown
    }
  }

private:
  std::string data_dir_param_;
  double log_hz_{10.0};

  std::string topic_image_;
  std::string topic_cmo_;
  std::string topic_vc_;
  std::string topic_errors_xyz_;

  fs::path base_data_dir_;
  fs::path runs_dir_;
  fs::path run_dir_;
  fs::path rates_csv_path_;
  fs::path rates_png_path_;
  fs::path tracking_png_path_;
  fs::path cpo_yaml_path_;

  std::ofstream csv_;
  SteadyClock::time_point start_;

  std::mutex mutex_;
  std::unordered_map<std::string, SlidingRate1s> rates_;
  std::vector<Sample6> vc_samples_;
  std::vector<Sample6> error_xyz_samples_;
  std::vector<Sample2> error_norm_samples_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::SubscriptionBase::SharedPtr sub_cmo_;
  rclcpp::SubscriptionBase::SharedPtr sub_vc_;
  rclcpp::SubscriptionBase::SharedPtr sub_errors_xyz_;

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