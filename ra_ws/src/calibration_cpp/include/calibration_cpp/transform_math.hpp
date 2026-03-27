#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <array>
#include <vector>

namespace calibration_cpp
{
Eigen::Matrix4d pose_stamped_to_T(const geometry_msgs::msg::PoseStamped& msg);
Eigen::Matrix4d tf_stamped_to_T(const geometry_msgs::msg::TransformStamped& msg);

Eigen::Matrix4d T_inv(const Eigen::Matrix4d& T);

std::array<double,6> T_to_pose6(const Eigen::Matrix4d& T);
Eigen::Matrix4d pose6_to_T(const std::array<double,6>& p);

Eigen::Matrix4d solve_AX_XB(const std::vector<Eigen::Matrix4d>& A,
                            const std::vector<Eigen::Matrix4d>& B);

void compute_AX_XB_residuals(const std::vector<Eigen::Matrix4d>& A,
                             const std::vector<Eigen::Matrix4d>& B,
                             const Eigen::Matrix4d& X,
                             double& mean_trans_m,
                             double& max_trans_m,
                             double& mean_rot_rad,
                             double& max_rot_rad);
}  // namespace calibration_cpp