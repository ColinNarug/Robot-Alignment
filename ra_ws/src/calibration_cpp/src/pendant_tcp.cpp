#include <rclcpp/rclcpp.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace tcp_math
{
Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& M)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

  if (R.determinant() < 0.0)
  {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U * svd.matrixV().transpose();
  }

  return R;
}

Eigen::Vector3d canonicalizePiCase(const Eigen::Vector3d& rvec)
{
  const double theta = rvec.norm();
  if (std::abs(theta - M_PI) > 1e-9)
    return rvec;

  Eigen::Vector3d out = rvec;
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(out(i)) > 1e-12)
    {
      if (out(i) < 0.0)
        out = -out;
      break;
    }
  }
  return out;
}

Eigen::Vector3d rotationMatrixToUrRotationVector(const Eigen::Matrix3d& R_input)
{
  const Eigen::Matrix3d R = projectToSO3(R_input);

  cv::Mat R_cv(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      R_cv.at<double>(r, c) = R(r, c);

  cv::Mat rvec_cv(3, 1, CV_64F);
  cv::Rodrigues(R_cv, rvec_cv);

  Eigen::Vector3d rvec;
  rvec(0) = rvec_cv.at<double>(0, 0);
  rvec(1) = rvec_cv.at<double>(1, 0);
  rvec(2) = rvec_cv.at<double>(2, 0);

  return canonicalizePiCase(rvec);
}
}

class TcpRotationVectorNode : public rclcpp::Node
{
public:
  TcpRotationVectorNode() : Node("pendant_tcp")
  {
    const double x_mm = this->declare_parameter<double>("x_mm", -161.0);
    const double y_mm = this->declare_parameter<double>("y_mm", -7.0);
    const double z_mm = this->declare_parameter<double>("z_mm", 102.0);

    const std::vector<double> R_param = this->declare_parameter<std::vector<double>>(
      "R",
      std::vector<double>{
        -1.0, 0.0,  0.0,
         0.0, 0.0, -1.0,
         0.0,-1.0,  0.0
      });

    if (R_param.size() != 9)
    {
      throw std::runtime_error("Parameter R must contain exactly 9 values.");
    }

    Eigen::Matrix3d R;
    R << R_param[0], R_param[1], R_param[2],
         R_param[3], R_param[4], R_param[5],
         R_param[6], R_param[7], R_param[8];

    const Eigen::Vector3d rvec = tcp_math::rotationMatrixToUrRotationVector(R);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(10);
    oss << "\nUR teach pendant TCP settings:\n"
        << "X  = " << x_mm << " mm\n"
        << "Y  = " << y_mm << " mm\n"
        << "Z  = " << z_mm << " mm\n"
        << "RX = " << rvec(0) << " rad\n"
        << "RY = " << rvec(1) << " rad\n"
        << "RZ = " << rvec(2) << " rad\n";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpRotationVectorNode>());
  rclcpp::shutdown();
  return 0;
}