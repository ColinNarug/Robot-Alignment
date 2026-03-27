#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <array>

namespace calibration_cpp
{
struct WriteTargets
{
  std::vector<std::string> install_dirs; // .../install/<pkg>/share/<pkg>/config
  std::vector<std::string> src_dirs;     // .../<ws>/src/<pkg>/config
};

WriteTargets compute_write_targets(const std::vector<std::string>& pkgs,
                                   const std::string& subdir,
                                   const std::string& workspace_root_hint);

void write_pose6_yaml_multi(const WriteTargets& targets,
                            const std::string& filename,
                            const std::array<double,6>& pose6);

bool read_pose6_yaml_first_found(const std::vector<std::string>& pkgs,
                                 const std::string& subdir,
                                 const std::string& workspace_root_hint,
                                 const std::string& filename,
                                 std::array<double,6>& out_pose6,
                                 std::string& out_path);

void write_intrinsics_yaml_multi(const WriteTargets& targets,
                                 const std::string& filename,
                                 int width, int height,
                                 const cv::Mat& K,
                                 const cv::Mat& D,
                                 const std::string& distortion_model,
                                 const std::string& camera_name);
}  // namespace calibration_cpp