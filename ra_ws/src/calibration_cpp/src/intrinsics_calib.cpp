/*
Keys:
  c = capture current frame (detect chessboard; store corners if found)
  s = solve intrinsics + write YAML + quality report bundle
  r = reset dataset
  q / ESC = quit

Outputs:
  1) Standard intrinsics YAML written to output_dir and mirror_output_dirs
  2) Session bundle under <workspace>/src/calibration_cpp/data/<timestamped_session_dir>/ containing:
     - 01_uncorrected_with_lines/
     - 02_corner_detection/
     - 03_corrected_image/
     - <serial>_<width>x<height>_intrinsics.yaml
     - calibration_quality_report.md
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

class IntrinsicsCalibNode final : public rclcpp::Node
{
public:
  explicit IntrinsicsCalibNode(const rclcpp::NodeOptions &options)
  : Node("intrinsics_calib_node", options)
  {
    // Parameters (overridable by params file):
    image_topic_         = this->declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    board_cols_          = this->declare_parameter<int>("board_cols", 24); // inner corners
    board_rows_          = this->declare_parameter<int>("board_rows", 17); // inner corners
    square_size_m_       = this->declare_parameter<double>("square_size_m", 0.0075); // 7.5 mm
    serial_              = this->declare_parameter<std::string>("serial", "");
    output_dir_          = this->declare_parameter<std::string>("output_dir", ""); // empty -> auto-resolve
    data_root_dir_       = this->declare_parameter<std::string>("data_root_dir", ""); // ignored at runtime
    mirror_output_dirs_  = this->declare_parameter<std::vector<std::string>>(
      "mirror_output_dirs", std::vector<std::string>{});
    preview_scale_       = this->declare_parameter<double>("preview_scale", 0.5);
    gui_period_ms_       = this->declare_parameter<int>("gui_period_ms", 33);

    // Detection controls:
    continuous_detect_hz_ = this->declare_parameter<double>("continuous_detect_hz", 15.0); // 0 -> disabled
    detect_scale_         = this->declare_parameter<double>("detect_scale", 0.35); // 0.25-1.0
    use_sb_               = this->declare_parameter<bool>("use_findChessboardCornersSB", true);
    adaptive_thresh_      = this->declare_parameter<bool>("adaptive_thresh", true);
    normalize_image_      = this->declare_parameter<bool>("normalize_image", true);
    fast_check_           = this->declare_parameter<bool>("fast_check", false);
    refine_corners_       = this->declare_parameter<bool>("refine_corners", true);
    refine_win_           = this->declare_parameter<int>("refine_window", 11);
    refine_max_iters_     = this->declare_parameter<int>("refine_max_iters", 30);
    refine_eps_           = this->declare_parameter<double>("refine_eps", 0.1);
    min_samples_for_solve_= this->declare_parameter<int>("min_samples_for_solve", 0);

    // Validation / report controls:
    undistort_alpha_      = this->declare_parameter<double>("undistort_alpha", 1.0);
    crop_undistorted_     = this->declare_parameter<bool>("crop_undistorted", true);
    write_session_bundle_ = this->declare_parameter<bool>("write_session_bundle", true);

    // Validate / normalize parameters:
    if (board_cols_ <= 1 || board_rows_ <= 1)
      throw std::runtime_error("board_cols and board_rows must be > 1 (inner corners).");
    if (square_size_m_ <= 0.0)
      throw std::runtime_error("square_size_m must be > 0.");
    if (detect_scale_ <= 0.0 || detect_scale_ > 1.0)
      detect_scale_ = 1.0;
    undistort_alpha_ = std::clamp(undistort_alpha_, 0.0, 1.0);

    board_size_ = cv::Size(board_cols_, board_rows_);

    // Object points for one board observation
    board_object_points_.reserve(static_cast<size_t>(board_cols_ * board_rows_));
    for (int r = 0; r < board_rows_; ++r)
    {
      for (int c = 0; c < board_cols_; ++c)
      {
        board_object_points_.emplace_back(
          static_cast<float>(c * square_size_m_),
          static_cast<float>(r * square_size_m_),
          0.0f
        );
      }
    }

    // Default output dir: uralignment_cpp/share/config so d435i_camera will find it
    if (output_dir_.empty())
    {
      try
      {
        output_dir_ = ament_index_cpp::get_package_share_directory("uralignment_cpp") + "/config";
      }
      catch (...)
      {
        output_dir_ = std::filesystem::current_path().string();
      }
    }

    data_root_dir_ = (resolve_calibration_cpp_source_root() / "data").string();

    // Callback groups:
    cbg_img_     = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbg_compute_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // QoS: match camera publisher best_effort depth=1
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cbg_img_;

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      std::bind(&IntrinsicsCalibNode::image_cb, this, std::placeholders::_1),
      sub_opts
    );

    compute_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&IntrinsicsCalibNode::compute_loop, this),
      cbg_compute_
    );

    RCLCPP_INFO(this->get_logger(),
      "Started. image_topic=%s | board=%dx%d | square_size_m=%.6f | detect_scale=%.2f | output_dir=%s | data_root_dir=%s | serial=%s",
      image_topic_.c_str(),
      board_cols_, board_rows_,
      square_size_m_,
      detect_scale_,
      output_dir_.c_str(),
      data_root_dir_.c_str(),
      serial_.c_str());

    RCLCPP_INFO(this->get_logger(),
      "Keys (GUI window): c=capture  s=solve+save  r=reset  q/ESC=quit");
  }

  void run_gui_main_thread()
  {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

    auto next_tick = std::chrono::steady_clock::now();

    while (rclcpp::ok())
    {
      next_tick += std::chrono::milliseconds(gui_period_ms_);

      sensor_msgs::msg::Image::ConstSharedPtr msg;
      {
        std::scoped_lock lk(img_mtx_);
        msg = latest_image_;
      }

      cv::Mat display;

      if (msg)
      {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
          if (msg->encoding == sensor_msgs::image_encodings::BGR8)
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
          else
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (...)
        {
          display = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
          cv::putText(display, "cv_bridge conversion failed",
                      cv::Point(15, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                      cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
          cv::imshow(window_name_, display);
          const int key = (cv::waitKey(1) & 0xFF);
          handle_keypress(key, msg);
          std::this_thread::sleep_until(next_tick);
          continue;
        }

        const cv::Mat &frame = cv_ptr->image;
        if (!frame.empty())
        {
          {
            std::scoped_lock lk(meta_mtx_);
            last_image_size_ = frame.size();
          }

          const double scale = preview_scale_;
          if (scale > 0.0 && scale < 1.0)
            cv::resize(frame, display, cv::Size(), scale, scale, cv::INTER_AREA);
          else
            display = frame;

          bool last_found = false;
          std::vector<cv::Point2f> corners;
          {
            std::scoped_lock lk(overlay_mtx_);
            last_found = last_found_;
            corners = last_corners_;
          }

          if (!corners.empty())
          {
            std::vector<cv::Point2f> scaled = corners;
            if (scale > 0.0 && scale < 1.0)
            {
              for (auto &p : scaled)
              {
                p.x = static_cast<float>(p.x * scale);
                p.y = static_cast<float>(p.y * scale);
              }
            }
            cv::drawChessboardCorners(display, board_size_, scaled, last_found);
          }

          int captured = 0;
          {
            std::scoped_lock lk(dataset_mtx_);
            captured = static_cast<int>(image_points_.size());
          }

          const std::string status =
            "captured: " + std::to_string(captured) +
            " | last_found: " + std::string(last_found ? "YES" : "NO") +
            " | keys: c capture, s solve, r reset, q quit";

          cv::putText(display, status, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX,
                      0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        }
      }
      else
      {
        display = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::putText(display, "Waiting for image on: " + image_topic_,
                    cv::Point(15, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      }

      cv::imshow(window_name_, display);
      const int key = (cv::waitKey(1) & 0xFF);
      handle_keypress(key, msg);

      std::this_thread::sleep_until(next_tick);
    }

    try { cv::destroyWindow(window_name_); } catch (...) {}
  }

private:
  struct CapturedSample
  {
    cv::Mat bgr;
    std::vector<cv::Point2f> corners;
  };

  struct LineResidualStats
  {
    double rms_px{0.0};
    double max_px{0.0};
    size_t n_points{0};
  };

  struct BorderStraightnessSummary
  {
    LineResidualStats top;
    LineResidualStats bottom;
    LineResidualStats left;
    LineResidualStats right;

    double mean_rms_px() const
    {
      return (top.rms_px + bottom.rms_px + left.rms_px + right.rms_px) / 4.0;
    }

    double mean_max_px() const
    {
      return (top.max_px + bottom.max_px + left.max_px + right.max_px) / 4.0;
    }
  };

  struct SampleReprojectionStats
  {
    double mean_px{0.0};
    double max_px{0.0};
  };

  struct SessionQualityAggregate
  {
    double mean_raw_border_rms_px{0.0};
    double mean_corrected_border_rms_px{0.0};
    double mean_border_improvement_px{0.0};
    double mean_border_improvement_percent{0.0};
    double mean_sample_reproj_px{0.0};
    double max_sample_reproj_px{0.0};
    double mean_opencv_per_view_error_px{0.0};
    double median_opencv_per_view_error_px{0.0};
    double max_opencv_per_view_error_px{0.0};
  };

  static std::filesystem::path resolve_calibration_cpp_source_root()
  {
    namespace fs = std::filesystem;

    // Preferred: compile-time source file location. For a normal colcon build
    // this resolves to:
    //   <workspace>/src/calibration_cpp/src/intrinsics_calib.cpp
    // so parent_path().parent_path() is:
    //   <workspace>/src/calibration_cpp
    try
    {
      const fs::path from_file = fs::path(__FILE__).lexically_normal().parent_path().parent_path();
      if (!from_file.empty() && fs::exists(from_file) && fs::exists(from_file / "config"))
        return from_file;
    }
    catch (...)
    {
    }

    // Fallback: search upward from the current working directory for
    // "src/calibration_cpp/config". This supports running from the workspace
    // root such as:
    //   ~/Robot-Alignment/ra_ws
    try
    {
      fs::path p = fs::current_path().lexically_normal();
      while (!p.empty())
      {
        const fs::path candidate = p / "src" / "calibration_cpp";
        if (fs::exists(candidate / "config"))
          return candidate;

        if (p == p.root_path()) break;
        p = p.parent_path();
      }
    }
    catch (...)
    {
    }

    // Last resort: keep the current working directory behavior but still
    // place the bundle in a sibling "data" directory.
    return fs::current_path();
  }

  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::scoped_lock lk(img_mtx_);
    latest_image_ = msg;
  }

  void handle_keypress(int key, const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (key == -1) return;

    if (key == 27 || key == 'q')
    {
      rclcpp::shutdown();
      return;
    }

    if (key == 'r')
    {
      reset_dataset();
      return;
    }

    if (key == 'c')
    {
      if (!msg) return;
      {
        std::scoped_lock lk(pending_mtx_);
        pending_image_ = msg;
        have_pending_.store(true, std::memory_order_release);
      }
      capture_requested_.store(true, std::memory_order_release);
      return;
    }

    if (key == 's')
    {
      solve_requested_.store(true, std::memory_order_release);
      return;
    }
  }

  void compute_loop()
  {
    if (solve_requested_.exchange(false, std::memory_order_acq_rel))
    {
      size_t n = 0;
      {
        std::scoped_lock lk(dataset_mtx_);
        n = image_points_.size();
      }
      RCLCPP_INFO(this->get_logger(), "Solve requested. Attempting calibration using %zu samples...", n);
      solve_and_write();
    }

    const auto now = this->now();
    const bool do_capture = capture_requested_.exchange(false, std::memory_order_acq_rel);

    bool do_continuous = false;
    if (!do_capture && continuous_detect_hz_ > 0.0)
    {
      const double period_s = 1.0 / continuous_detect_hz_;
      if ((now - last_continuous_time_).seconds() >= period_s)
      {
        do_continuous = true;
        last_continuous_time_ = now;
      }
    }

    if (!do_capture && !do_continuous) return;

    sensor_msgs::msg::Image::ConstSharedPtr msg;
    if (do_capture)
    {
      std::scoped_lock lk(pending_mtx_);
      if (!have_pending_.load(std::memory_order_acquire) || !pending_image_) return;
      msg = pending_image_;
      pending_image_.reset();
      have_pending_.store(false, std::memory_order_release);
    }
    else
    {
      std::scoped_lock lk(img_mtx_);
      msg = latest_image_;
    }

    if (!msg) return;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      else
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (...)
    {
      return;
    }

    if (cv_ptr->image.empty()) return;

    cv::Mat gray_full;
    cv::cvtColor(cv_ptr->image, gray_full, cv::COLOR_BGR2GRAY);

    cv::Mat gray = gray_full;
    const double s = detect_scale_;
    if (s > 0.0 && s < 1.0)
      cv::resize(gray_full, gray, cv::Size(), s, s, cv::INTER_AREA);

    std::vector<cv::Point2f> corners;
    bool found = false;

    if (use_sb_)
    {
      found = cv::findChessboardCornersSB(gray, board_size_, corners);
    }
    else
    {
      int flags = 0;
      if (adaptive_thresh_) flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
      if (normalize_image_) flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
      if (fast_check_)      flags |= cv::CALIB_CB_FAST_CHECK;
      found = cv::findChessboardCorners(gray, board_size_, corners, flags);
    }

    if (found && (s > 0.0 && s < 1.0))
    {
      const double inv = 1.0 / s;
      for (auto &p : corners)
      {
        p.x = static_cast<float>(p.x * inv);
        p.y = static_cast<float>(p.y * inv);
      }
    }

    if (found && refine_corners_)
    {
      const int w = std::max(3, refine_win_);
      const cv::TermCriteria term(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
        std::max(1, refine_max_iters_), refine_eps_);
      cv::cornerSubPix(gray_full, corners, cv::Size(w, w), cv::Size(-1, -1), term);
    }

    {
      std::scoped_lock lk(overlay_mtx_);
      last_found_ = found;
      last_corners_ = corners;
    }

    if (!do_capture) return;

    if (!found)
    {
      RCLCPP_WARN(this->get_logger(), "Capture: chessboard NOT found.");
      return;
    }

    {
      std::scoped_lock lk(dataset_mtx_);
      image_points_.push_back(corners);
      object_points_.push_back(board_object_points_);
      captured_samples_.push_back(CapturedSample{cv_ptr->image.clone(), corners});
      RCLCPP_INFO(this->get_logger(), "Capture OK: stored sample %zu.", image_points_.size());
    }
  }

  void solve_and_write()
  {
    std::vector<std::vector<cv::Point2f>> img_pts;
    std::vector<std::vector<cv::Point3f>> obj_pts;
    std::vector<CapturedSample> samples;
    {
      std::scoped_lock lk(dataset_mtx_);
      img_pts = image_points_;
      obj_pts = object_points_;
      samples = captured_samples_;
    }

    const int min_needed = (min_samples_for_solve_ <= 0) ? 3 : std::max(3, min_samples_for_solve_);
    if (static_cast<int>(img_pts.size()) < min_needed)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Solve requested but only %zu samples collected (need at least %d).",
                  img_pts.size(), min_needed);
      return;
    }

    cv::Size img_size;
    {
      std::scoped_lock lk(meta_mtx_);
      img_size = last_image_size_;
    }
    if (img_size.width <= 0 || img_size.height <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "Solve requested but image size is unknown.");
      return;
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat stddev_intrinsics, stddev_extrinsics, per_view_errors;

    double rms = 0.0;
    try
    {
      rms = cv::calibrateCamera(
        obj_pts, img_pts, img_size,
        K, D, rvecs, tvecs,
        stddev_intrinsics, stddev_extrinsics, per_view_errors,
        0);
    }
    catch (const cv::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "calibrateCamera failed: %s", e.what());
      return;
    }

    const double reproj_mean_px = compute_mean_reprojection_error(obj_pts, img_pts, rvecs, tvecs, K, D);

    std::string serial = sanitize_component(serial_);
    if (serial.empty()) serial = "UNKNOWN_SERIAL";

    const std::string yaml_filename =
      serial + "_" + std::to_string(img_size.width) + "x" + std::to_string(img_size.height) + "_intrinsics.yaml";

    // Primary and mirror config outputs.
    std::unordered_set<std::string> seen;
    std::vector<std::string> dirs;
    dirs.reserve(1 + mirror_output_dirs_.size());

    auto add_dir = [&](const std::string &d)
    {
      if (d.empty()) return;
      if (seen.insert(d).second) dirs.push_back(d);
    };

    add_dir(output_dir_);
    for (const auto &d : mirror_output_dirs_) add_dir(d);

    std::vector<std::string> wrote_paths;
    wrote_paths.reserve(dirs.size());

    for (const auto &dir : dirs)
    {
      const std::filesystem::path p = std::filesystem::path(dir) / yaml_filename;
      try { std::filesystem::create_directories(p.parent_path()); } catch (...) {}

      if (write_yaml(p.string(), K, D, img_size, rms, reproj_mean_px))
      {
        wrote_paths.push_back(p.string());
        RCLCPP_INFO(this->get_logger(), "Wrote intrinsics YAML: %s", p.string().c_str());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to write intrinsics YAML: %s", p.string().c_str());
      }
    }

    if (wrote_paths.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write intrinsics YAML to all output directories.");
      return;
    }

    std::filesystem::path session_dir;
    SessionQualityAggregate aggregate;

    if (write_session_bundle_)
    {
      try
      {
        session_dir = write_session_bundle(
          samples, obj_pts, img_pts, rvecs, tvecs,
          K, D, img_size,
          rms, reproj_mean_px,
          per_view_errors, stddev_intrinsics,
          yaml_filename, serial,
          aggregate);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to write session bundle: %s", e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Calibration complete.\n"
                "  RMS (OpenCV): %.6f\n"
                "  Mean reprojection error: %.6f px\n"
                "  Wrote %zu config YAML file(s)%s%s",
                rms,
                reproj_mean_px,
                wrote_paths.size(),
                session_dir.empty() ? "" : "\n  Session bundle: ",
                session_dir.empty() ? "" : session_dir.string().c_str());
  }

  std::filesystem::path write_session_bundle(
    const std::vector<CapturedSample> &samples,
    const std::vector<std::vector<cv::Point3f>> &obj_pts,
    const std::vector<std::vector<cv::Point2f>> &img_pts,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const cv::Mat &K,
    const cv::Mat &D,
    const cv::Size &img_size,
    double opencv_rms,
    double mean_reprojection_error_px,
    const cv::Mat &per_view_errors,
    const cv::Mat &stddev_intrinsics,
    const std::string &yaml_filename,
    const std::string &serial,
    SessionQualityAggregate &aggregate_out)
  {
    const std::filesystem::path data_root(data_root_dir_);
    std::filesystem::create_directories(data_root);

    const std::string timestamp = wall_timestamp_string();
    const std::string session_name =
      "intrinsics_" + timestamp + "_" + serial + "_" +
      std::to_string(img_size.width) + "x" + std::to_string(img_size.height);

    const std::filesystem::path session_dir = data_root / session_name;
    const std::filesystem::path raw_lines_dir = session_dir / "01_uncorrected_with_lines";
    const std::filesystem::path corners_dir   = session_dir / "02_corner_detection";
    const std::filesystem::path corrected_dir = session_dir / "03_corrected_image";

    std::filesystem::create_directories(raw_lines_dir);
    std::filesystem::create_directories(corners_dir);
    std::filesystem::create_directories(corrected_dir);

    const std::filesystem::path yaml_path = session_dir / yaml_filename;
    if (!write_yaml(yaml_path.string(), K, D, img_size, opencv_rms, mean_reprojection_error_px))
      throw std::runtime_error("Could not write session YAML: " + yaml_path.string());

    cv::Rect roi;
    const cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, img_size, undistort_alpha_, img_size, &roi);

    std::vector<double> raw_border_rms_values;
    std::vector<double> corrected_border_rms_values;
    std::vector<double> border_improvement_values;
    std::vector<double> border_improvement_percent_values;
    std::vector<double> sample_reproj_mean_values;
    double max_sample_reproj_px = 0.0;

    const size_t n = std::min({samples.size(), img_pts.size(), obj_pts.size(), rvecs.size(), tvecs.size()});

    std::ofstream report(session_dir / "calibration_quality_report.md");
    if (!report.is_open())
      throw std::runtime_error("Could not open calibration quality report for writing.");

    report << "# Calibration Quality Report\n\n";
    report << "- Session folder: `" << session_name << "`\n";
    report << "- Camera serial: `" << serial << "`\n";
    report << "- Image size: `" << img_size.width << " x " << img_size.height << "`\n";
    report << "- Board inner corners: `" << board_cols_ << " x " << board_rows_ << "`\n";
    report << "- Square size: `" << square_size_m_ << " m`\n";
    report << "- Captured samples used: `" << n << "`\n";
    report << "- OpenCV RMS: `" << opencv_rms << " px`\n";
    report << "- Mean reprojection error: `" << mean_reprojection_error_px << " px`\n";
    report << "- Undistort alpha: `" << undistort_alpha_ << "`\n";
    report << "- Crop undistorted image: `" << (crop_undistorted_ ? "true" : "false") << "`\n\n";

    report << "## Intrinsics\n\n";
    report << "```text\n";
    report << "K =\n" << mat_to_string(K) << "\n\n";
    report << "D =\n" << mat_to_string(D) << "\n";
    report << "```\n\n";

    if (!stddev_intrinsics.empty())
    {
      report << "## Intrinsic Parameter Standard Deviations\n\n";
      report << "```text\n" << mat_to_string(stddev_intrinsics) << "\n```\n\n";
    }

    const std::vector<double> opencv_per_view = mat_to_vector(per_view_errors);
    const auto opencv_per_view_stats = compute_scalar_stats(opencv_per_view);

    report << "## Aggregate Quantitative Checks\n\n";
    report << "- OpenCV per-view reprojection error mean: `" << opencv_per_view_stats.mean << " px`\n";
    report << "- OpenCV per-view reprojection error median: `" << opencv_per_view_stats.median << " px`\n";
    report << "- OpenCV per-view reprojection error max: `" << opencv_per_view_stats.max << " px`\n\n";

    report << "## Per-Sample Checks\n\n";
    report << "| Sample | Reproj mean (px) | Reproj max (px) | Raw border RMS (px) | Corrected border RMS (px) | Border improvement (px) | Improvement (%) | Raw lines | Corners | Corrected |\n";
    report << "|---:|---:|---:|---:|---:|---:|---:|---|---|---|\n";

    for (size_t i = 0; i < n; ++i)
    {
      const std::string sample_name = make_sample_name(i);
      const auto &sample = samples[i];

      cv::Mat raw_with_lines = sample.bgr.clone();
      if (!raw_with_lines.empty())
        draw_outer_border_lines(raw_with_lines, sample.corners, board_size_);

      cv::Mat corners_img = sample.bgr.clone();
      if (!corners_img.empty())
        cv::drawChessboardCorners(corners_img, board_size_, sample.corners, true);

      cv::Mat corrected_img;
      if (!sample.bgr.empty())
        corrected_img = undistort_for_report(sample.bgr, K, D, newK, roi);

      std::vector<cv::Point2f> corrected_corners;
      if (!sample.corners.empty())
      {
        cv::undistortPoints(sample.corners, corrected_corners, K, D, cv::noArray(), newK);
        if (crop_undistorted_ && roi.width > 0 && roi.height > 0)
        {
          for (auto &p : corrected_corners)
          {
            p.x -= static_cast<float>(roi.x);
            p.y -= static_cast<float>(roi.y);
          }
        }
      }

      if (!corrected_img.empty() && !corrected_corners.empty())
        draw_outer_border_lines(corrected_img, corrected_corners, board_size_);

      const BorderStraightnessSummary raw_border = compute_border_straightness(sample.corners, board_size_);
      const BorderStraightnessSummary corrected_border = compute_border_straightness(corrected_corners, board_size_);
      const SampleReprojectionStats reproj_stats =
        compute_sample_reprojection_stats(obj_pts[i], img_pts[i], rvecs[i], tvecs[i], K, D);

      const double border_improvement_px = raw_border.mean_rms_px() - corrected_border.mean_rms_px();
      const double border_improvement_pct =
        (raw_border.mean_rms_px() > std::numeric_limits<double>::epsilon())
          ? (100.0 * border_improvement_px / raw_border.mean_rms_px())
          : 0.0;

      raw_border_rms_values.push_back(raw_border.mean_rms_px());
      corrected_border_rms_values.push_back(corrected_border.mean_rms_px());
      border_improvement_values.push_back(border_improvement_px);
      border_improvement_percent_values.push_back(border_improvement_pct);
      sample_reproj_mean_values.push_back(reproj_stats.mean_px);
      max_sample_reproj_px = std::max(max_sample_reproj_px, reproj_stats.max_px);

      const std::filesystem::path raw_path = raw_lines_dir / (sample_name + ".png");
      const std::filesystem::path corner_path = corners_dir / (sample_name + ".png");
      const std::filesystem::path corrected_path = corrected_dir / (sample_name + ".png");

      if (!raw_with_lines.empty()) cv::imwrite(raw_path.string(), raw_with_lines);
      if (!corners_img.empty()) cv::imwrite(corner_path.string(), corners_img);
      if (!corrected_img.empty()) cv::imwrite(corrected_path.string(), corrected_img);

      report << "| " << (i + 1)
             << " | " << reproj_stats.mean_px
             << " | " << reproj_stats.max_px
             << " | " << raw_border.mean_rms_px()
             << " | " << corrected_border.mean_rms_px()
             << " | " << border_improvement_px
             << " | " << border_improvement_pct
             << " | `" << raw_path.filename().string()
             << "` | `" << corner_path.filename().string()
             << "` | `" << corrected_path.filename().string()
             << "` |\n";
    }

    const auto raw_stats = compute_scalar_stats(raw_border_rms_values);
    const auto corrected_stats = compute_scalar_stats(corrected_border_rms_values);
    const auto improvement_stats = compute_scalar_stats(border_improvement_values);
    const auto improvement_pct_stats = compute_scalar_stats(border_improvement_percent_values);
    const auto sample_reproj_stats = compute_scalar_stats(sample_reproj_mean_values);

    aggregate_out.mean_raw_border_rms_px = raw_stats.mean;
    aggregate_out.mean_corrected_border_rms_px = corrected_stats.mean;
    aggregate_out.mean_border_improvement_px = improvement_stats.mean;
    aggregate_out.mean_border_improvement_percent = improvement_pct_stats.mean;
    aggregate_out.mean_sample_reproj_px = sample_reproj_stats.mean;
    aggregate_out.max_sample_reproj_px = max_sample_reproj_px;
    aggregate_out.mean_opencv_per_view_error_px = opencv_per_view_stats.mean;
    aggregate_out.median_opencv_per_view_error_px = opencv_per_view_stats.median;
    aggregate_out.max_opencv_per_view_error_px = opencv_per_view_stats.max;

    report << "\n## Aggregate Straightness Check\n\n";
    report << "- Mean raw border RMS: `" << raw_stats.mean << " px`\n";
    report << "- Mean corrected border RMS: `" << corrected_stats.mean << " px`\n";
    report << "- Mean border straightness improvement: `" << improvement_stats.mean << " px`\n";
    report << "- Mean border straightness improvement: `" << improvement_pct_stats.mean << " %`\n\n";

    report << "## Qualitative Interpretation\n\n";
    report << "The validation bundle is intended to answer two questions:\n\n";
    report << "1. **Do the fitted parameters reproduce the observed corner locations well?**  \n";
    report << "   This is summarized by the OpenCV RMS value, the mean reprojection error, and the per-sample reprojection statistics.\n\n";
    report << "2. **Do straight physical chessboard borders become straighter after undistortion?**  \n";
    report << "   This is summarized by the raw-versus-corrected border-line RMS residuals and the saved validation images.\n\n";

    if (improvement_stats.mean > 0.0)
    {
      report << "Observed result: the corrected images reduce the mean border-line residual by `"
             << improvement_stats.mean << " px` on average (`"
             << improvement_pct_stats.mean << " %` improvement).\n";
    }
    else if (improvement_stats.mean < 0.0)
    {
      report << "Observed result: the corrected images increased the border-line residual by `"
             << std::abs(improvement_stats.mean) << " px` on average. Re-check the dataset.\n";
    }
    else
    {
      report << "Observed result: no measurable average change in border-line residual was detected.\n";
    }

    report << "\nUse the images in `01_uncorrected_with_lines`, `02_corner_detection`, and `03_corrected_image` together with the reprojection numbers above to judge calibration quality for this run.\n";

    report.close();
    return session_dir;
  }

  void reset_dataset()
  {
    {
      std::scoped_lock lk(dataset_mtx_);
      image_points_.clear();
      object_points_.clear();
      captured_samples_.clear();
    }
    {
      std::scoped_lock lk(overlay_mtx_);
      last_found_ = false;
      last_corners_.clear();
    }
    RCLCPP_INFO(this->get_logger(), "Dataset reset.");
  }

  static double compute_mean_reprojection_error(
    const std::vector<std::vector<cv::Point3f>> &obj_pts,
    const std::vector<std::vector<cv::Point2f>> &img_pts,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const cv::Mat &K,
    const cv::Mat &D)
  {
    double total_err = 0.0;
    size_t total_points = 0;

    for (size_t i = 0; i < obj_pts.size(); ++i)
    {
      std::vector<cv::Point2f> projected;
      cv::projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, D, projected);

      const auto &measured = img_pts[i];
      for (size_t j = 0; j < measured.size() && j < projected.size(); ++j)
      {
        const cv::Point2f d = measured[j] - projected[j];
        total_err += std::sqrt(d.x * d.x + d.y * d.y);
      }
      total_points += measured.size();
    }

    if (total_points == 0) return 0.0;
    return total_err / static_cast<double>(total_points);
  }

  static SampleReprojectionStats compute_sample_reprojection_stats(
    const std::vector<cv::Point3f> &obj_pts,
    const std::vector<cv::Point2f> &img_pts,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    const cv::Mat &K,
    const cv::Mat &D)
  {
    SampleReprojectionStats out;
    if (obj_pts.empty() || img_pts.empty()) return out;

    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj_pts, rvec, tvec, K, D, projected);

    double sum = 0.0;
    double max_v = 0.0;
    const size_t n = std::min(img_pts.size(), projected.size());
    for (size_t i = 0; i < n; ++i)
    {
      const cv::Point2f d = img_pts[i] - projected[i];
      const double e = std::sqrt(d.x * d.x + d.y * d.y);
      sum += e;
      max_v = std::max(max_v, e);
    }

    out.mean_px = (n > 0) ? (sum / static_cast<double>(n)) : 0.0;
    out.max_px = max_v;
    return out;
  }

  static std::vector<cv::Point2f> get_row_points(
    const std::vector<cv::Point2f> &corners, int row, int cols)
  {
    std::vector<cv::Point2f> out;
    if (cols <= 0 || row < 0) return out;
    const size_t start = static_cast<size_t>(row * cols);
    if (start + static_cast<size_t>(cols) > corners.size()) return out;
    out.insert(out.end(), corners.begin() + static_cast<long>(start), corners.begin() + static_cast<long>(start + cols));
    return out;
  }

  static std::vector<cv::Point2f> get_col_points(
    const std::vector<cv::Point2f> &corners, int col, int rows, int cols)
  {
    std::vector<cv::Point2f> out;
    if (col < 0 || cols <= 0 || rows <= 0) return out;
    out.reserve(static_cast<size_t>(rows));
    for (int r = 0; r < rows; ++r)
    {
      const size_t idx = static_cast<size_t>(r * cols + col);
      if (idx >= corners.size()) break;
      out.push_back(corners[idx]);
    }
    return out;
  }

  static LineResidualStats compute_line_residual_stats(const std::vector<cv::Point2f> &pts)
  {
    LineResidualStats out;
    if (pts.size() < 2) return out;

    cv::Vec4f line;
    cv::fitLine(pts, line, cv::DIST_L2, 0.0, 0.01, 0.01);

    const double vx = static_cast<double>(line[0]);
    const double vy = static_cast<double>(line[1]);
    const double x0 = static_cast<double>(line[2]);
    const double y0 = static_cast<double>(line[3]);

    double sum_sq = 0.0;
    double max_abs = 0.0;
    for (const auto &p : pts)
    {
      const double x = static_cast<double>(p.x);
      const double y = static_cast<double>(p.y);
      const double dist = std::abs(vy * (x - x0) - vx * (y - y0));
      sum_sq += dist * dist;
      max_abs = std::max(max_abs, dist);
    }

    out.rms_px = std::sqrt(sum_sq / static_cast<double>(pts.size()));
    out.max_px = max_abs;
    out.n_points = pts.size();
    return out;
  }

  static BorderStraightnessSummary compute_border_straightness(
    const std::vector<cv::Point2f> &corners,
    const cv::Size &board_size)
  {
    BorderStraightnessSummary out;
    if (board_size.width <= 1 || board_size.height <= 1) return out;
    if (corners.size() != static_cast<size_t>(board_size.width * board_size.height)) return out;

    const auto top = get_row_points(corners, 0, board_size.width);
    const auto bottom = get_row_points(corners, board_size.height - 1, board_size.width);
    const auto left = get_col_points(corners, 0, board_size.height, board_size.width);
    const auto right = get_col_points(corners, board_size.width - 1, board_size.height, board_size.width);

    out.top = compute_line_residual_stats(top);
    out.bottom = compute_line_residual_stats(bottom);
    out.left = compute_line_residual_stats(left);
    out.right = compute_line_residual_stats(right);
    return out;
  }

  static void draw_outer_border_lines(
    cv::Mat &image,
    const std::vector<cv::Point2f> &corners,
    const cv::Size &board_size)
  {
    if (image.empty()) return;
    if (corners.size() != static_cast<size_t>(board_size.width * board_size.height)) return;

    const cv::Point2f &tl = corners.front();
    const cv::Point2f &tr = corners[static_cast<size_t>(board_size.width - 1)];
    const cv::Point2f &bl = corners[static_cast<size_t>((board_size.height - 1) * board_size.width)];
    const cv::Point2f &br = corners.back();

    const cv::Scalar red(0, 0, 255);
    const int thickness = 2;
    cv::line(image, tl, tr, red, thickness, cv::LINE_AA);
    cv::line(image, tr, br, red, thickness, cv::LINE_AA);
    cv::line(image, br, bl, red, thickness, cv::LINE_AA);
    cv::line(image, bl, tl, red, thickness, cv::LINE_AA);
  }

  cv::Mat undistort_for_report(
    const cv::Mat &src,
    const cv::Mat &K,
    const cv::Mat &D,
    const cv::Mat &newK,
    const cv::Rect &roi) const
  {
    cv::Mat dst;
    cv::undistort(src, dst, K, D, newK);
    if (crop_undistorted_ && roi.width > 0 && roi.height > 0)
      return dst(roi).clone();
    return dst;
  }

  static bool write_yaml(const std::string &path,
                         const cv::Mat &K,
                         const cv::Mat &D,
                         const cv::Size &img_size,
                         double rms,
                         double reproj_mean_px)
  {
    try
    {
      cv::FileStorage fs(path, cv::FileStorage::WRITE);
      if (!fs.isOpened()) return false;

      fs << "image_width"  << img_size.width;
      fs << "image_height" << img_size.height;
      fs << "distortion_model" << "plumb_bob";
      fs << "camera_matrix" << K;
      fs << "distortion_coefficients" << D;
      fs << "opencv_rms" << rms;
      fs << "mean_reprojection_error_px" << reproj_mean_px;

      fs.release();
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  struct ScalarStats
  {
    double mean{0.0};
    double median{0.0};
    double min{0.0};
    double max{0.0};
  };

  static ScalarStats compute_scalar_stats(const std::vector<double> &values)
  {
    ScalarStats out;
    if (values.empty()) return out;

    out.mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());

    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());

    out.min = sorted.front();
    out.max = sorted.back();
    if (sorted.size() % 2 == 0)
      out.median = 0.5 * (sorted[sorted.size() / 2 - 1] + sorted[sorted.size() / 2]);
    else
      out.median = sorted[sorted.size() / 2];

    return out;
  }

  static std::vector<double> mat_to_vector(const cv::Mat &m)
  {
    std::vector<double> out;
    if (m.empty()) return out;

    cv::Mat flat = m.reshape(1, 1);
    cv::Mat flat64;
    flat.convertTo(flat64, CV_64F);
    out.reserve(flat64.cols);
    for (int i = 0; i < flat64.cols; ++i)
      out.push_back(flat64.at<double>(0, i));
    return out;
  }

  static std::string mat_to_string(const cv::Mat &m)
  {
    std::ostringstream oss;
    cv::Mat m64;
    m.convertTo(m64, CV_64F);
    oss << std::fixed << std::setprecision(8);
    for (int r = 0; r < m64.rows; ++r)
    {
      for (int c = 0; c < m64.cols; ++c)
      {
        oss << std::setw(14) << m64.at<double>(r, c);
        if (c + 1 < m64.cols) oss << ' ';
      }
      if (r + 1 < m64.rows) oss << '\n';
    }
    return oss.str();
  }

  static std::string sanitize_component(const std::string &s)
  {
    std::string out;
    out.reserve(s.size());
    for (const char ch : s)
    {
      if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '-' || ch == '_')
        out.push_back(ch);
      else
        out.push_back('_');
    }
    return out;
  }

  static std::string wall_timestamp_string()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::string make_sample_name(size_t idx)
  {
    std::ostringstream oss;
    oss << "sample_" << std::setw(4) << std::setfill('0') << (idx + 1);
    return oss.str();
  }

private:
  // Parameters
  std::string image_topic_;
  int board_cols_{24};
  int board_rows_{17};
  double square_size_m_{0.0075};
  std::string serial_;
  std::string output_dir_;
  std::string data_root_dir_;
  std::vector<std::string> mirror_output_dirs_;

  double preview_scale_{0.5};
  int gui_period_ms_{33};

  double continuous_detect_hz_{2.0};
  rclcpp::Time last_continuous_time_{0, 0, RCL_ROS_TIME};
  double detect_scale_{0.5};

  bool use_sb_{true};
  bool adaptive_thresh_{true};
  bool normalize_image_{true};
  bool fast_check_{false};

  bool refine_corners_{true};
  int refine_win_{11};
  int refine_max_iters_{30};
  double refine_eps_{0.1};
  int min_samples_for_solve_{0};

  double undistort_alpha_{1.0};
  bool crop_undistorted_{true};
  bool write_session_bundle_{true};

  // Board model
  cv::Size board_size_;
  std::vector<cv::Point3f> board_object_points_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr cbg_img_;
  rclcpp::CallbackGroup::SharedPtr cbg_compute_;

  // ROS entities
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr compute_timer_;

  // Latest image
  std::mutex img_mtx_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;

  // Pending capture
  std::mutex pending_mtx_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
  std::atomic<bool> capture_requested_{false};
  std::atomic<bool> have_pending_{false};

  // Overlay
  std::mutex overlay_mtx_;
  bool last_found_{false};
  std::vector<cv::Point2f> last_corners_;

  // Dataset
  std::mutex dataset_mtx_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  std::vector<std::vector<cv::Point3f>> object_points_;
  std::vector<CapturedSample> captured_samples_;

  // Metadata
  std::mutex meta_mtx_;
  cv::Size last_image_size_{0, 0};

  // Solve request
  std::atomic<bool> solve_requested_{false};

  // GUI
  const std::string window_name_{"intrinsics_calib"};
};

static rclcpp::NodeOptions make_node_options_with_default_params()
{
  rclcpp::NodeOptions options;

  try
  {
    const std::string share = ament_index_cpp::get_package_share_directory("calibration_cpp");
    const std::string params = share + "/config/intrinsics_calib.params.yaml";

    if (std::filesystem::exists(params))
    {
      options.arguments(std::vector<std::string>{
        "--ros-args", "--params-file", params
      });
    }
  }
  catch (...)
  {
    // If share dir isn't found, run with code defaults.
  }

  return options;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto options = make_node_options_with_default_params();
  auto node = std::make_shared<IntrinsicsCalibNode>(options);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);

  std::thread spin_thread([&exec]() { exec.spin(); });

  node->run_gui_main_thread();

  exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
