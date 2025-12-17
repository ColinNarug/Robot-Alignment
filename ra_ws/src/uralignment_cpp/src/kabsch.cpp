#include <Eigen/Geometry>
#include <visp3/vision/vpPose.h>
#include "kabsch.h"

// Giffen added for ROS2:
#include <string> // String Operations
#include "rclcpp/rclcpp.hpp" // ROS2 CPP API
#include "std_msgs/msg/string.hpp" // ROS2 String Message
#include "sensor_msgs/msg/image.hpp" // ROS2 Image Message


// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
// The input 3D points are stored as columns.
Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;
    
  //  printf("scale %lf\n", scale);

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}

Eigen::Affine3d Find3DAffineTransformSameScale(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out)
{
    // Default output
    Eigen::Affine3d A;
    A.linear() = Eigen::Matrix3d::Identity(3, 3);
    A.translation() = Eigen::Vector3d::Zero();
    
    if (in.cols() != out.cols())
        throw "Find3DAffineTransform(): input data mis-match";
    
    // Find the centroids then shift to the origin
    Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
    Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
    for (int col = 0; col < in.cols(); col++) {
        in_ctr  += in.col(col);
        out_ctr += out.col(col);
    }
    in_ctr /= in.cols();
    out_ctr /= out.cols();
    for (int col = 0; col < in.cols(); col++) {
        in.col(col)  -= in_ctr;
        out.col(col) -= out_ctr;
    }
    
    // SVD
    Eigen::MatrixXd Cov = in * out.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Find the rotation
    double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > 0)
        d = 1.0;
    else
        d = -1.0;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(2, 2) = d;
    Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();
    
    // The final transform
    A.linear() = R;
    A.translation() = (out_ctr - R*in_ctr);
    
    return A;
}

// VisP wrapper
vpHomogeneousMatrix findPose(std::vector<vpColVector> in, std::vector<vpColVector> out) { //TODO sanity checks on dimensions
  // Create an Eigen Matrix to hold the points
  Eigen::Matrix3Xd inputPoints(3, in.size()), outputPoints(3, out.size());

  // Fill the Eigen matrix with the coordinates from vpColVector
  for (size_t i = 0; i < in.size(); ++i) {
      inputPoints(0, i) = in[i][0];
      inputPoints(1, i) = in[i][1];
      inputPoints(2, i) = in[i][2];
  }
  for (size_t i = 0; i < out.size(); ++i) {
      outputPoints(0, i) = out[i][0];
      outputPoints(1, i) = out[i][1];
      outputPoints(2, i) = out[i][2];
  }

  Eigen::Affine3d A = Find3DAffineTransformSameScale(inputPoints, outputPoints);


  vpHomogeneousMatrix cMo;
  cMo[0][0] = A.linear()(0, 0); cMo[0][1] = A.linear()(0, 1); cMo[0][2] = A.linear()(0, 2); cMo[0][3] = A.translation()(0);
  cMo[1][0] = A.linear()(1, 0); cMo[1][1] = A.linear()(1, 1); cMo[1][2] = A.linear()(1, 2); cMo[1][3] = A.translation()(1);
  cMo[2][0] = A.linear()(2, 0); cMo[2][1] = A.linear()(2, 1); cMo[2][2] = A.linear()(2, 2); cMo[2][3] = A.translation()(2);
  return cMo;
}

