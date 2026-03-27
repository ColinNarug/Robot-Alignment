#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#endif
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpPose.h>
#include <visp3/gui/vpPlot.h>
#include <string>                    // String Operations
#include "rclcpp/rclcpp.hpp"         // ROS2 CPP API
#include "sensor_msgs/msg/image.hpp" // ROS2 Image Message
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp> // File Paths for .yamls
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace fs = std::filesystem;


#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party(X11, GDI or openCV) to dislay images..." << std::endl;
#else
  bool display_off = false;
#endif

class DisplayNode : public rclcpp::Node
{
public:
  DisplayNode() : Node("displays_node")
  {

    image_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cam_opts;
    cam_opts.callback_group = image_handling_;
    vector_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions vector_opts;
    vector_opts.callback_group = vector_handling_;
    plot_handling_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions plot_opts;
    plot_opts.callback_group = plot_handling_;

    // Parameters
    width_ = declare_parameter<int>("width",1920);
    height_ = declare_parameter<int>("height",1080);
    fps_ = declare_parameter<int>("fps",30);
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
    // Subscribe to Camera node's topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", qos,
    std::bind(&DisplayNode::image_callback, this, std::placeholders::_1), cam_opts);

    cMo_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "cMo", 10, std::bind(&DisplayNode::cMo_callback, this, std::placeholders::_1), vector_opts);

    v_c_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "v_c", 10, std::bind(&DisplayNode::v_c_callback, this, std::placeholders::_1), vector_opts);

    errors_xyz_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "errors_xyz", 10, std::bind(&DisplayNode::errors_xyz_callback, this, std::placeholders::_1), vector_opts);

    error_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "error", 10, std::bind(&DisplayNode::error_callback, this, std::placeholders::_1), vector_opts);

    // timer for plot_loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
    std::bind(&DisplayNode::plot_callback, this), plot_handling_);

    initialization();
  }

private:
  rclcpp::CallbackGroup::SharedPtr image_handling_, vector_handling_, plot_handling_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr v_c_, errors_xyz_, error_;  
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr cMo_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;

  // Params
  int width_{1920};
  int height_{1080};
  int fps_{30};
  int qos_depth_{5};
  std::string qos_reliability_{"best_effort"};
  std::string preferred_encoding_{"bgr8"};

  std::string package_path = ament_index_cpp::get_package_share_directory("calibration_cpp");
  std::string cdMo_filename = package_path + "/config/cdPo.yaml";
  bool opt_plot;
  bool display_initialized_;
  vpColVector v_c_data_, error_data_, errors_xyz_data_, v_local, exyz_local, err_local;
  vpPoseVector cdPo;
  vpHomogeneousMatrix cdMo, cdMc, cMo_data_, cMo_local;
  vpTranslationVector cMo_t, cd_t_c;
  vpQuaternionVector cMo_q;
  vpRotationMatrix cMo_R;
  // Define ViSP image buffers matching RealSense camera resolution:
  vpImage<vpRGBa> I_color;
  vpImage<unsigned char> I;
  vpPlot *plotter = nullptr;
  int iter_plot;
  vpCameraParameters cam;
  std::stringstream ss;
  std::vector<vpImagePoint> *traj_vip = nullptr; // To memorize point trajectory
  double t_start;
  std::unique_ptr<vpDisplay> disp_;

  std::mutex img_mtx_;
  bool have_img_{false};
  std::mutex state_mtx_;
  bool have_cam_model_{false};
  std::string active_camera_path_;
  std::string intrinsics_path_;
  std::string active_serial_;
  int active_width_{0};
  int active_height_{0};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};

  static std::string trim_(const std::string& s)
  {
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
  }

  static std::string infer_workspace_root_from_share_(const std::string& share_dir)
  {
    // Expect something like: <ws>/install/<pkg>/share/<pkg>
    const std::string needle = "/install/";
    const auto pos = share_dir.find(needle);
    if (pos == std::string::npos) return "";
    return share_dir.substr(0, pos);
  }

  std::vector<std::string> candidate_config_dirs_()
  {
    std::vector<std::string> dirs;
    dirs.push_back(package_path + "/config");

    const std::string ws_root = infer_workspace_root_from_share_(package_path);
    if (!ws_root.empty())
    {
      dirs.push_back(ws_root + "/config");
      dirs.push_back(ws_root + "/src/uralignment_cpp/config");
      dirs.push_back(ws_root + "/src/calibration_cpp/config");
      dirs.push_back(ws_root + "/src/uralignment_py/config");
      dirs.push_back(ws_root + "/install/uralignment_cpp/share/uralignment_cpp/config");
      dirs.push_back(ws_root + "/install/calibration_cpp/share/calibration_cpp/config");
      dirs.push_back(ws_root + "/install/uralignment_py/share/uralignment_py/config");
    }

    // de-dup while preserving order
    std::vector<std::string> uniq;
    uniq.reserve(dirs.size());
    for (const auto& d : dirs)
    {
      if (d.empty()) continue;
      if (std::find(uniq.begin(), uniq.end(), d) == uniq.end())
        uniq.push_back(d);
    }
    return uniq;
  }

  std::vector<std::string> candidate_intrinsics_dirs_()
  {
    auto dirs = candidate_config_dirs_();
    const std::string ws_root = infer_workspace_root_from_share_(package_path);
    if (!ws_root.empty())
      dirs.push_back(ws_root + "/config/intrinsics");

    // de-dup
    std::vector<std::string> uniq;
    uniq.reserve(dirs.size());
    for (const auto& d : dirs)
    {
      if (d.empty()) continue;
      if (std::find(uniq.begin(), uniq.end(), d) == uniq.end())
        uniq.push_back(d);
    }
    return uniq;
  }

  static std::string find_first_existing_(const std::vector<std::string>& dirs, const std::string& filename)
  {
    for (const auto& d : dirs)
    {
      std::error_code ec;
      const fs::path p = fs::path(d) / filename;
      if (fs::exists(p, ec) && !ec)
        return p.string();
    }
    return "";
  }

  static void ensure_dir_(const std::string& dir)
  {
    std::error_code ec;
    fs::create_directories(dir, ec);
  }

  // Reads dPo.yaml format:
  // rows: 6
  // cols: 1
  // data:
  //   - [val]
  static bool read_posevector_yaml_exact_(const std::string& path, vpPoseVector& out)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    int rows = 0, cols = 0;
    std::vector<double> values;
    std::string line;
    bool in_data = false;

    while (std::getline(ifs, line))
    {
      line = trim_(line);
      if (line.empty()) continue;
      if (line.rfind("%YAML", 0) == 0) continue;
      if (line == "---") continue;
      if (line[0] == '#') continue;

      if (!in_data)
      {
        if (line.rfind("rows:", 0) == 0) rows = std::stoi(trim_(line.substr(5)));
        else if (line.rfind("cols:", 0) == 0) cols = std::stoi(trim_(line.substr(5)));
        else if (line.rfind("data:", 0) == 0) in_data = true;
      }
      else
      {
        const auto lb = line.find('[');
        const auto rb = line.find(']');
        if (lb == std::string::npos || rb == std::string::npos || rb <= lb + 1) continue;
        const std::string num = trim_(line.substr(lb + 1, rb - lb - 1));
        try { values.push_back(std::stod(num)); } catch (...) {}
      }
    }

    if (rows != 6 || cols != 1 || values.size() < 6) return false;

    // ViSP expects resize(rows, cols, flagNullify)
    vpPoseVector tmp;  // fixed-size 6x1
    for (int i = 0; i < 6; ++i) tmp[i] = values[(size_t)i];
    out = tmp;
    return true;
  }

  // Writes exact cdPo.yaml format
  static bool write_posevector_yaml_exact_(const std::string& path, const vpPoseVector& p)
  {
    std::ofstream ofs(path, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) return false;

    ofs << "%YAML:1.0\n---\n";
    ofs << "rows: 6\n";
    ofs << "cols: 1\n";
    ofs << "data:\n";
    for (int i = 0; i < 6; ++i)
      ofs << "  - [" << std::setprecision(17) << p[i] << "]\n";
    return true;
  }

  bool write_cdPo_to_all_config_dirs_(const vpPoseVector& p)
  {
    const auto dirs = candidate_config_dirs_();
    bool wrote_any = false;

    for (const auto& d : dirs)
    {
      try
      {
        ensure_dir_(d);
        const fs::path out = fs::path(d) / "cdPo.yaml";
        if (write_posevector_yaml_exact_(out.string(), p))
          wrote_any = true;
      }
      catch (...) {}
    }

    return wrote_any;
  }

  bool load_camera_intrinsics_from_yaml_()
  {
    const auto cfg_dirs = candidate_config_dirs_();
    const auto intr_dirs = candidate_intrinsics_dirs_();

    const std::string ac_path = find_first_existing_(cfg_dirs, "active_camera.yaml");
    if (ac_path.empty()) return false;

    std::string serial;
    int w = 0, h = 0;

    // Read with OpenCV FileStorage
    cv::FileStorage fsr(ac_path, cv::FileStorage::READ);
    if (!fsr.isOpened()) return false;
    fsr["serial"] >> serial;
    fsr["width"]  >> w;
    fsr["height"] >> h;
    fsr.release();

    if (serial.empty() || w <= 0 || h <= 0) return false;

    const std::string intr_name = serial + "_" + std::to_string(w) + "x" + std::to_string(h) + "_intrinsics.yaml";
    const std::string intr_path = find_first_existing_(intr_dirs, intr_name);
    if (intr_path.empty()) return false;

    cv::Mat K, D;
    cv::FileStorage fsi(intr_path, cv::FileStorage::READ);
    if (!fsi.isOpened()) return false;

    if (!fsi["camera_matrix"].empty()) fsi["camera_matrix"] >> K;
    if (K.empty() && !fsi["K"].empty()) fsi["K"] >> K;

    if (!fsi["distortion_coefficients"].empty()) fsi["distortion_coefficients"] >> D;
    if (D.empty() && !fsi["D"].empty()) fsi["D"] >> D;

    fsi.release();

    if (K.empty() || K.rows != 3 || K.cols != 3) return false;

    cam.initPersProjWithoutDistortion(
      K.at<double>(0,0),
      K.at<double>(1,1),
      K.at<double>(0,2),
      K.at<double>(1,2));

    fx_ = K.at<double>(0,0);
    fy_ = K.at<double>(1,1);
    cx_ = K.at<double>(0,2);
    cy_ = K.at<double>(1,2);

    // commit info for printing/debug
    active_camera_path_ = ac_path;
    intrinsics_path_ = intr_path;
    active_serial_ = serial;
    active_width_ = w;
    active_height_ = h;

    have_cam_model_ = true;
    return true;
  }

  bool load_desired_pose_from_yaml_()
  {
    const auto cfg_dirs = candidate_config_dirs_();

    // Prefer canonical cdPo.yaml anywhere
    std::string desired_path = find_first_existing_(cfg_dirs, "cdPo.yaml");

    if (!desired_path.empty())
    {
      if (read_posevector_yaml_exact_(desired_path, cdPo))
      {
        cdMo.buildFrom(cdPo);
        std::cout << "Read Desired Transformation from file: " << desired_path << "\ncdMo:\n" << cdMo << "\n";
        return true;
      }

      // Fallback: try ViSP loader
      try {
        cdPo.loadYAML(desired_path, cdPo);
        cdMo.buildFrom(cdPo);
        std::cout << "Read Desired Transformation from file (ViSP loadYAML): " << desired_path << "\ncdMo:\n" << cdMo << "\n";
        return true;
      } catch (...) {
        std::cout << "Found cdPo.yaml but failed to parse it: " << desired_path << "\n";
      }
    }

    desired_path = find_first_existing_(cfg_dirs, "ur_cdMo.yaml");
    if (!desired_path.empty())
    {
      cdPo.loadYAML(desired_path, cdPo);
      cdMo.buildFrom(cdPo);
      std::cout << "Read Desired Transformation from legacy file: " << desired_path << "\ncdMo:\n" << cdMo << "\n";

      write_cdPo_to_all_config_dirs_(cdPo);

      return true;
    }

    // Nothing found: set identity desired pose
    for (int i = 0; i < 6; ++i) cdPo[i] = 0.0;
    cdMo = vpHomogeneousMatrix();
    std::cout << "WARNING: cdPo.yaml and ur_cdMo.yaml not found. Using identity desired pose.\n";
    return false;
  }

  void capture_cdPo_from_current_cMo_(const std::string& reason)
  {
    // Convert current pose to pose vector and write it out
    vpPoseVector cPo(cMo_local);

    if (!write_cdPo_to_all_config_dirs_(cPo))
    {
      std::cout << "ERROR: Failed to write cdPo.yaml (reason: " << reason << ")\n";
      return;
    }

    // Update desired pose immediately so overlay updates without restart
    cdPo = cPo;
    cdMo.buildFrom(cdPo);

    std::cout << "Saved cdPo.yaml from current cMo (reason: " << reason << ")\n";
  }

  void initialization()
  {
    if (!load_camera_intrinsics_from_yaml_())
    {
      have_cam_model_ = false;
      std::cout << "WARNING: Could not load camera intrinsics yet. Waiting for active_camera.yaml and intrinsics yaml...\n";
    }
    else
    {
      std::cout << "Loaded intrinsics from yaml:\n";
      std::cout << "  active_camera: " << active_camera_path_ << "\n";
      std::cout << "  intrinsics:    " << intrinsics_path_ << "\n";
      std::cout << "  serial/res:    " << active_serial_ << " " << active_width_ << "x" << active_height_ << "\n";
    }

    // Build Desired Object relative to Camera Frame:
    load_desired_pose_from_yaml_();

    iter_plot = 0;
    I_color.resize(height_, width_);
    I.resize(height_, width_);
    plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
    opt_plot = true;
    t_start = vpTime::measureTimeMs();

    if(opt_plot)
    {
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      plotter->initGraph(0, 6);
      plotter->initGraph(1, 6);
      plotter->setLegend(0, 0, "error_feat_tx");
      plotter->setLegend(0, 1, "error_feat_ty");
      plotter->setLegend(0, 2, "error_feat_tz");
      plotter->setLegend(0, 3, "error_feat_theta_ux");
      plotter->setLegend(0, 4, "error_feat_theta_uy");
      plotter->setLegend(0, 5, "error_feat_theta_uz");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
    }

    display_initialized_ = false;

    cMo_t = vpTranslationVector(0.0, 0.0, 0.0);
    cMo_q = vpQuaternionVector(0.0, 0.0, 0.0, 1.0);
    cMo_R.buildFrom(cMo_q);
    cMo_data_.buildFrom(cMo_t, cMo_R);

    v_c_data_.resize(6); v_c_data_ = 0;
    errors_xyz_data_.resize(6); errors_xyz_data_ = 0;
    error_data_.resize(2); error_data_ = 0;


    std::cout << "Displays Node Started!" << std::endl;
    std::cout << "Controls: Focus the \"Pose from Homography\" window and press 'c' to capture cdPo.yaml" << std::endl;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg) return;
    std::scoped_lock lk(img_mtx_);
    latest_image_ = msg;
    have_img_ = true;
  }

  void cMo_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    if(!msg) return;
    std::scoped_lock lk(state_mtx_);
    const auto& T = msg->transform;
    cMo_t[0] = T.translation.x;
    cMo_t[1] = T.translation.y;
    cMo_t[2] = T.translation.z;
    cMo_q[0] = T.rotation.x;
    cMo_q[1] = T.rotation.y;
    cMo_q[2] = T.rotation.z;
    cMo_q[3] = T.rotation.w;
    cMo_R.buildFrom(cMo_q);
    cMo_data_.buildFrom(cMo_t, cMo_R);
  }

  void v_c_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() != 6)
      return;
    std::scoped_lock lk(state_mtx_);
    for (int i = 0; i < 6; ++i)
    {
      v_c_data_[i] = msg->data[i];
    }
  }

  void errors_xyz_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() != 6)
      return;
    std::scoped_lock lk(state_mtx_);
    for (int i = 0; i < 6; ++i)
    {
      errors_xyz_data_[i] = msg->data[i];
    }
  }

  void error_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() != 2)
      return;
    std::scoped_lock lk(state_mtx_);
    for (int i = 0; i < 2; ++i)
    {
      error_data_[i] = msg->data[i];
    }
  }

  void plot_callback()
  {
    sensor_msgs::msg::Image::ConstSharedPtr img;
    {
      std::scoped_lock lk(img_mtx_);
      if (!have_img_ || !latest_image_) return;
      img = latest_image_;
    }

    {
      std::scoped_lock lk(state_mtx_);
      cMo_local = cMo_data_;
      v_local = v_c_data_;
      exyz_local = errors_xyz_data_;
      err_local = error_data_;
    }
    cdMc = cdMo * cMo_local.inverse(); // Update visual features
    if (!have_cam_model_)
    {
      if (load_camera_intrinsics_from_yaml_())
      {
        std::cout << "Loaded intrinsics from yaml (late):\n";
        std::cout << "  active_camera: " << active_camera_path_ << "\n";
        std::cout << "  intrinsics:    " << intrinsics_path_ << "\n";
      }
    }

    // Convertion (GUI thread)
    cv_bridge::CvImageConstPtr cv_ptr;
    try 
    {
      cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat &mat = cv_ptr->image;
    if (mat.empty()) return;

    cv::Mat annotated = mat.clone();
    if (have_cam_model_)
    {
      const double X = cMo_local[0][3];
      const double Y = cMo_local[1][3];
      const double Z = cMo_local[2][3];
      if (std::isfinite(X) && std::isfinite(Y) && std::isfinite(Z) && Z > 1e-9)
      {
        const int u = static_cast<int>(std::lround(fx_ * (X / Z) + cx_));
        const int v = static_cast<int>(std::lround(fy_ * (Y / Z) + cy_));
        if (u >= 0 && u < annotated.cols && v >= 0 && v < annotated.rows)
        {
          cv::drawMarker(
            annotated,
            cv::Point(u, v),
            cv::Scalar(0, 255, 255),
            cv::MARKER_CROSS,
            24,
            2,
            cv::LINE_AA);
          cv::circle(
            annotated,
            cv::Point(u, v),
            10,
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
          cv::putText(
            annotated,
            "tag center",
            cv::Point(u + 14, std::max(20, v - 14)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
        }
      }
    }

    // Ensure dimensions match before writing
    const int h = annotated.rows, w = annotated.cols;
    if ((int)I_color.getHeight() != h || (int)I_color.getWidth() != w) 
    {
      I_color.resize(h, w);
      I.resize(h, w);
      display_initialized_ = false;
    }

    // Convert mat -> I_color
    for (int i=0;i<h;++i)
      for (int j=0;j<w;++j) 
      {
        const cv::Vec3b &p = annotated.at<cv::Vec3b>(i,j);
        I_color[i][j] = vpRGBa(p[2], p[1], p[0]);
      }

    if (!display_initialized_) 
    {
#ifdef VISP_HAVE_X11
      disp_.reset(); // close previous (if any)
      disp_ = std::make_unique<vpDisplayX>(I_color, 100, 30, "Pose from Homography");
      display_initialized_ = true;
#else
      return; // no GUI backend available
#endif
    }

    try
    {
      if (disp_)
      {
        vpDisplay::display(I_color);
        vpImageConvert::convert(I_color, I);
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
        // Display desired and current pose features:
        if (have_cam_model_)
        {
          vpDisplay::displayFrame(I_color, cdMo, cam, 0.06512, vpColor::yellow, 2);
          vpDisplay::displayFrame(I_color, cMo_local, cam, 0.06512, vpColor::none, 3);
        }

        vpDisplay::flush(I_color);

        // Press 'c' in the ViSP window to capture cdPo.yaml
        std::string key;
        if (vpDisplay::getKeyboardEvent(I_color, key, false))
        {
          if (key == "c" || key == "C")
          {
            capture_cdPo_from_current_cMo_("keypress 'c'");
          }
        }
      
      }

      if (opt_plot)
      {
        plotter->plot(0, iter_plot, exyz_local);
        plotter->plot(1, iter_plot, v_local);
        iter_plot++;
      }

      ss.str("");
      ss << "error_t: " << err_local[0];
      vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
      ss.str("");
      ss << "error_tu: " << err_local[1];
      vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
      ss.str("");
      ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";

      vpDisplay::displayText(I_color, 40, 20, ss.str(), vpColor::red);
      vpDisplay::flush(I_color);
    }
    catch (const vpException &e)
    {
      std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}