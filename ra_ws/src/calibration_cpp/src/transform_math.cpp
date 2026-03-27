#include "calibration_cpp/transform_math.hpp"
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <cmath>

namespace calibration_cpp
{
Eigen::Matrix4d pose_stamped_to_T(const geometry_msgs::msg::PoseStamped& msg)
{
  const auto &p = msg.pose.position;
  const auto &q = msg.pose.orientation;
  Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
  Q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = Q.toRotationMatrix();
  T(0,3)=p.x; T(1,3)=p.y; T(2,3)=p.z;
  return T;
}

Eigen::Matrix4d tf_stamped_to_T(const geometry_msgs::msg::TransformStamped& msg)
{
  const auto &t = msg.transform.translation;
  const auto &q = msg.transform.rotation;
  Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
  Q.normalize();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = Q.toRotationMatrix();
  T(0,3)=t.x; T(1,3)=t.y; T(2,3)=t.z;
  return T;
}

Eigen::Matrix4d T_inv(const Eigen::Matrix4d& T)
{
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3,3>(0,0);
  const Eigen::Vector3d p = T.block<3,1>(0,3);
  Ti.block<3,3>(0,0) = R.transpose();
  Ti.block<3,1>(0,3) = -R.transpose()*p;
  return Ti;
}

std::array<double,6> T_to_pose6(const Eigen::Matrix4d& T)
{
  std::array<double,6> p{};
  p[0] = T(0,3); p[1] = T(1,3); p[2] = T(2,3);

  cv::Mat R(3,3,CV_64F);
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) R.at<double>(r,c)=T(r,c);
  cv::Mat rvec(3,1,CV_64F);
  cv::Rodrigues(R, rvec);
  p[3] = rvec.at<double>(0);
  p[4] = rvec.at<double>(1);
  p[5] = rvec.at<double>(2);
  return p;
}

Eigen::Matrix4d pose6_to_T(const std::array<double,6>& p)
{
  cv::Mat rvec(3,1,CV_64F);
  rvec.at<double>(0)=p[3];
  rvec.at<double>(1)=p[4];
  rvec.at<double>(2)=p[5];

  cv::Mat R(3,3,CV_64F);
  cv::Rodrigues(rvec, R);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) T(r,c)=R.at<double>(r,c);
  T(0,3)=p[0]; T(1,3)=p[1]; T(2,3)=p[2];
  return T;
}

static Eigen::Matrix3d project_to_SO3(const Eigen::Matrix3d& M)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU|Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU()*svd.matrixV().transpose();
  if(R.determinant()<0)
  {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U*svd.matrixV().transpose();
  }
  return R;
}

Eigen::Matrix4d solve_AX_XB(const std::vector<Eigen::Matrix4d>& A,
                            const std::vector<Eigen::Matrix4d>& B)
{
  if(A.size()!=B.size() || A.empty())
    throw std::runtime_error("solve_AX_XB: A and B must be same non-zero size");

  const int m = (int)A.size();
  Eigen::MatrixXd M(9*m, 9);
  M.setZero();

  for(int i=0;i<m;++i)
  {
    const Eigen::Matrix3d RA = A[i].block<3,3>(0,0);
    const Eigen::Matrix3d RB = B[i].block<3,3>(0,0);
    Eigen::Matrix<double,9,9> Ki; Ki.setZero();

    for(int k=0;k<3;++k) Ki.block<3,3>(3*k,3*k)=RA;

    const Eigen::Matrix3d RBT = RB.transpose();
    for(int r=0;r<3;++r)
      for(int c=0;c<3;++c)
        Ki.block<3,3>(3*r,3*c) -= RBT(r,c)*Eigen::Matrix3d::Identity();

    M.block(9*i,0,9,9)=Ki;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
  Eigen::VectorXd v = svd.matrixV().col(8);
  Eigen::Matrix3d RX;
  RX << v(0), v(3), v(6),
        v(1), v(4), v(7),
        v(2), v(5), v(8);
  RX = project_to_SO3(RX);

  Eigen::MatrixXd C(3*m,3);
  Eigen::VectorXd d(3*m);
  for(int i=0;i<m;++i){
    const Eigen::Matrix3d RA = A[i].block<3,3>(0,0);
    const Eigen::Vector3d tA = A[i].block<3,1>(0,3);
    const Eigen::Vector3d tB = B[i].block<3,1>(0,3);
    C.block<3,3>(3*i,0) = RA - Eigen::Matrix3d::Identity();
    d.segment<3>(3*i) = RX*tB - tA;
  }

  Eigen::Vector3d tX = C.bdcSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(d);

  Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
  X.block<3,3>(0,0)=RX;
  X.block<3,1>(0,3)=tX;
  return X;
}

static double rot_angle_from_R(const Eigen::Matrix3d& R)
{
  double tr = (R.trace() - 1.0) * 0.5;
  tr = std::max(-1.0, std::min(1.0, tr));
  return std::acos(tr);
}

void compute_AX_XB_residuals(const std::vector<Eigen::Matrix4d>& A,
                             const std::vector<Eigen::Matrix4d>& B,
                             const Eigen::Matrix4d& X,
                             double& mean_trans_m,
                             double& max_trans_m,
                             double& mean_rot_rad,
                             double& max_rot_rad)
{
  const size_t n = A.size();
  double sum_t = 0.0, sum_r = 0.0;
  double max_t = 0.0, max_r = 0.0;

  for (size_t i=0;i<n;++i)
  {
    Eigen::Matrix4d L = A[i] * X;
    Eigen::Matrix4d R = X * B[i];
    Eigen::Matrix4d E = T_inv(L) * R;

    const double t_err = E.block<3,1>(0,3).norm();
    const double r_err = rot_angle_from_R(E.block<3,3>(0,0));

    sum_t += t_err; sum_r += r_err;
    max_t = std::max(max_t, t_err);
    max_r = std::max(max_r, r_err);
  }

  mean_trans_m = sum_t / (double)n;
  mean_rot_rad = sum_r / (double)n;
  max_trans_m = max_t;
  max_rot_rad = max_r;
}
} // namespace calibration_cpp