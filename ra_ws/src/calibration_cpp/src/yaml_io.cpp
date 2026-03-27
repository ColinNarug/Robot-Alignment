#include "calibration_cpp/yaml_io.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/persistence.hpp>
#include <filesystem>
#include <stdexcept>

namespace fs = std::filesystem;

namespace calibration_cpp
{
static std::string infer_workspace_root_from_share(const std::string& share_dir)
{
  const std::string needle = "/install/";
  const auto pos = share_dir.find(needle);
  if (pos == std::string::npos) return "";
  return share_dir.substr(0, pos);
}

WriteTargets compute_write_targets(const std::vector<std::string>& pkgs,
                                   const std::string& subdir,
                                   const std::string& workspace_root_hint)
{
  WriteTargets t;

  for (const auto& pkg : pkgs)
  {
    try {
      const std::string share = ament_index_cpp::get_package_share_directory(pkg);
      const std::string install_cfg = share + "/" + subdir;
      fs::create_directories(install_cfg);
      t.install_dirs.push_back(install_cfg);

      // Infer workspace root from share unless explicitly provided
      std::string ws = workspace_root_hint;
      if (ws.empty()) ws = infer_workspace_root_from_share(share);

      if (!ws.empty()) {
        const std::string src_cfg = ws + "/src/" + pkg + "/" + subdir;
        fs::create_directories(src_cfg);
        t.src_dirs.push_back(src_cfg);
      }
    } catch (...) {}
  }

  return t;
}

static void write_pose6_yaml_one(const std::string& path, const std::array<double,6>& p)
{
  cv::FileStorage fsw(path, cv::FileStorage::WRITE);
  fsw << "rows" << 6;
  fsw << "cols" << 1;
  fsw << "data" << "[";
  for (int i=0;i<6;++i) fsw << "[" << p[(size_t)i] << "]";
  fsw << "]";
}

void write_pose6_yaml_multi(const WriteTargets& targets,
                            const std::string& filename,
                            const std::array<double,6>& pose6)
{
  for (const auto& d : targets.install_dirs) {
    write_pose6_yaml_one(d + "/" + filename, pose6);
  }
  for (const auto& d : targets.src_dirs) {
    write_pose6_yaml_one(d + "/" + filename, pose6);
  }
}

static bool read_pose6_yaml_one(const std::string& path, std::array<double,6>& out)
{
  cv::FileStorage fsr(path, cv::FileStorage::READ);
  if (!fsr.isOpened()) return false;

  int rows=0, cols=0;
  fsr["rows"] >> rows;
  fsr["cols"] >> cols;
  if (rows != 6 || cols != 1) return false;

  cv::FileNode data = fsr["data"];
  if (data.type() != cv::FileNode::SEQ) return false;

  int i = 0;
  for (auto it = data.begin(); it != data.end() && i < 6; ++it, ++i) {
    cv::FileNode row = *it;
    if (row.type() == cv::FileNode::SEQ && row.size() == 1) 
    {
      out[(size_t)i] = (double)row[0];
    } 
    else 
    {
      out[(size_t)i] = (double)row;
    }
  }
  return (i == 6);
}

bool read_pose6_yaml_first_found(const std::vector<std::string>& pkgs,
                                 const std::string& subdir,
                                 const std::string& workspace_root_hint,
                                 const std::string& filename,
                                 std::array<double,6>& out_pose6,
                                 std::string& out_path)
{
  // Prefer install/share first (consistent with ament_index usage)
  for (const auto& pkg : pkgs)
  {
    try {
      const std::string share = ament_index_cpp::get_package_share_directory(pkg);
      const std::string p = share + "/" + subdir + "/" + filename;
      if (fs::exists(p) && read_pose6_yaml_one(p, out_pose6)) 
      {
        out_path = p;
        return true;
      }
    } catch (...) {}
  }

  // Then try src mirrors
  for (const auto& pkg : pkgs)
  {
    try {
      const std::string share = ament_index_cpp::get_package_share_directory(pkg);
      std::string ws = workspace_root_hint;
      if (ws.empty()) ws = infer_workspace_root_from_share(share);
      if (ws.empty()) continue;

      const std::string p = ws + "/src/" + pkg + "/" + subdir + "/" + filename;
      if (fs::exists(p) && read_pose6_yaml_one(p, out_pose6)) 
      {
        out_path = p;
        return true;
      }
    } catch (...) {}
  }

  return false;
}

void write_intrinsics_yaml_multi(const WriteTargets& targets,
                                 const std::string& filename,
                                 int width, int height,
                                 const cv::Mat& K,
                                 const cv::Mat& D,
                                 const std::string& distortion_model,
                                 const std::string& camera_name)
{
  auto write_one = [&](const std::string& path)
  {
    cv::FileStorage fsw(path, cv::FileStorage::WRITE);

    fsw << "image_width" << width;
    fsw << "image_height" << height;
    fsw << "camera_name" << camera_name;
    fsw << "distortion_model" << distortion_model;

    fsw << "camera_matrix" << K;
    fsw << "distortion_coefficients" << D;

    fsw << "visp" << "{";
    fsw << "px" << K.at<double>(0,0);
    fsw << "py" << K.at<double>(1,1);
    fsw << "u0" << K.at<double>(0,2);
    fsw << "v0" << K.at<double>(1,2);

    fsw << "d" << "[";
    for (size_t i = 0; i < (size_t)D.total(); ++i) 
    {
      fsw << D.at<double>(0, static_cast<int>(i));
    }
    fsw << "]";
    fsw << "}";
  };

  for (const auto& d : targets.install_dirs) write_one(d + "/" + filename);
  for (const auto& d : targets.src_dirs)     write_one(d + "/" + filename);
}
} // namespace calibration_cpp