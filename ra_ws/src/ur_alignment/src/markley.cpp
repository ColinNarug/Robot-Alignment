#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <visp3/vision/vpPose.h>
#include "markley.h"
// Giffen added for ROS2:
#include <string> // String Operations
#include "rclcpp/rclcpp.hpp" // ROS2 CPP API
#include "std_msgs/msg/string.hpp" // ROS2 String Message
#include "sensor_msgs/msg/image.hpp" // ROS2 Image Message


// TODO Comments Markley
Eigen::Quaterniond findAverageQuaternion(const std::vector<Eigen::Quaterniond> &quaternions)
{
  if (quaternions.empty())
  {
    return Eigen::Quaterniond::Identity(); // Return identity if no quaternions provided
  }

  Eigen::Matrix4d covarianceMatrix = Eigen::Matrix4d::Zero(); // 4x4 covariance matrix

  // Accumulate the covariance matrix from each quaternion
  for (const auto &q : quaternions)
  {
    Eigen::Vector4d qVec(q.w(), q.x(), q.y(), q.z()); // Quaternion as a 4D vector
    covarianceMatrix += qVec * qVec.transpose();      // Outer product and accumulate
  }

  // Perform eigen decomposition on the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigenSolver(covarianceMatrix);

  // The eigenvector corresponding to the largest eigenvalue is the average quaternion
  Eigen::Vector4d avgQuaternionVec = eigenSolver.eigenvectors().col(3); // Largest eigenvalue (col 3)

  // Create a quaternion from the eigenvector
  Eigen::Quaterniond avgQuaternion(avgQuaternionVec(0), avgQuaternionVec(1), avgQuaternionVec(2), avgQuaternionVec(3));

  // Normalize the quaternion to ensure it's a valid rotation
  avgQuaternion.normalize();

  return avgQuaternion;
}

// Function to calculate the rotation distance (angle between two quaternions)
double rotationDistance(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2)
{
  return q1.angularDistance(q2) * (180.0 / M_PI); // Convert to degrees
}

// Function to calculate the rotation around the z-axis of the mean pose (clockwise distance)
double clockDistance(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, const Eigen::Vector3d &zAxis)
{
  // Project the rotation difference onto the z-axis to find the clock rotation component
  Eigen::Quaterniond relativeRotation = q1.conjugate() * q2;
  Eigen::AngleAxisd angleAxis(relativeRotation);
  double angle = angleAxis.angle() * angleAxis.axis().dot(zAxis);
  return std::abs(angle * (180.0 / M_PI)); // Convert to degrees
}

// Function to calculate the remaining rotation (parallel distance)
double parallelDistance(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, const Eigen::Vector3d &zAxis)
{
  Eigen::Quaterniond relativeRotation = q1.conjugate() * q2;
  Eigen::AngleAxisd angleAxis(relativeRotation);

  // Subtract the z-axis component to get the parallel component
  Eigen::Vector3d parallelAxis = angleAxis.axis() - angleAxis.axis().dot(zAxis) * zAxis;
  double parallelAngle = angleAxis.angle() * parallelAxis.norm();
  return std::abs(parallelAngle * (180.0 / M_PI)); // Convert to degrees
}

// Function to calculate the translation distance (Euclidean distance)
double translationDistance(const vpTranslationVector &t1, const vpTranslationVector &t2)
{
  return (Eigen::Vector3d(t1[0], t1[1], t1[2]) - Eigen::Vector3d(t2[0], t2[1], t2[2])).norm();
}

// VisP wrapper
vpHomogeneousMatrix optimalPose(std::vector<vpHomogeneousMatrix> inPoses)
{
  // Create appropriate data structures
  std::vector<Eigen::Quaterniond> quaternions;
  std::vector<vpTranslationVector> translations;

  // Extract translations and quaternions from input poses
  for (const auto &pose : inPoses)
  {
    vpRotationMatrix R;
    pose.extract(R);
    Eigen::Matrix3d rotationMatrix;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        rotationMatrix(i, j) = R[i][j];
    quaternions.push_back(Eigen::Quaterniond(rotationMatrix));

    vpTranslationVector t;
    pose.extract(t);
    translations.push_back(t);
  }

  // Find average quaternion (rotation) and average translation
  Eigen::Quaterniond avgQuaternion = findAverageQuaternion(quaternions);
  Eigen::Matrix3d avgRotationMatrix = avgQuaternion.toRotationMatrix();
  vpRotationMatrix avgVispRotation;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      avgVispRotation[i][j] = avgRotationMatrix(i, j);

  vpTranslationVector avgTranslation(0, 0, 0);
  for (const auto &t : translations)
  {
    avgTranslation[0] += t[0];
    avgTranslation[1] += t[1];
    avgTranslation[2] += t[2];
  }
  avgTranslation /= translations.size();

  vpHomogeneousMatrix avgPose(avgTranslation, avgVispRotation);
  Eigen::Vector3d zAxis = avgRotationMatrix.col(2); // Mean pose z-axis

  // Initialize accumulators for residuals
  double totalRotationResidual = 0.0;
  double totalClockResidual = 0.0;
  double totalParallelResidual = 0.0;
  double totalTranslationResidual = 0.0;
  double totalError = 0.0;

  // Vectors to store each residual for standard deviation calculation
  std::vector<double> rotationResiduals, clockResiduals, parallelResiduals, translationResiduals, errorResiduals;

  for (size_t i = 0; i < inPoses.size(); ++i)
  {
    // Compute the rotation and translation distances
    double rotDist = rotationDistance(quaternions[i], avgQuaternion);
    double clockDist = clockDistance(quaternions[i], avgQuaternion, zAxis);
    double parallelDist = parallelDistance(quaternions[i], avgQuaternion, zAxis);
    double transDist = translationDistance(translations[i], avgTranslation);
    double error = transDist + 0.0012 * clockDist + 0.006 * parallelDist;

    // Accumulate residuals
    rotationResiduals.push_back(rotDist);
    clockResiduals.push_back(clockDist);
    parallelResiduals.push_back(parallelDist);
    translationResiduals.push_back(transDist);
    errorResiduals.push_back(error);

    totalRotationResidual += rotDist;
    totalClockResidual += clockDist;
    totalParallelResidual += parallelDist;
    totalTranslationResidual += transDist;
    totalError += error;
  }

  // Calculate means
  double meanRotationResidual = totalRotationResidual / inPoses.size();
  double meanClockResidual = totalClockResidual / inPoses.size();
  double meanParallelResidual = totalParallelResidual / inPoses.size();
  double meanTranslationResidual = totalTranslationResidual / inPoses.size();
  double meanError = totalError / inPoses.size();

  // Calculate standard deviations
  auto calcStandardDeviation = [](const std::vector<double> &values, double mean)
  {
    double sumSquaredDiffs = 0.0;
    for (double value : values)
    {
      sumSquaredDiffs += (value - mean) * (value - mean);
    }
    return std::sqrt(sumSquaredDiffs / values.size());
  };

  double stdDevRotation = calcStandardDeviation(rotationResiduals, meanRotationResidual);
  double stdDevClock = calcStandardDeviation(clockResiduals, meanClockResidual);
  double stdDevParallel = calcStandardDeviation(parallelResiduals, meanParallelResidual);
  double stdDevTranslation = calcStandardDeviation(translationResiduals, meanTranslationResidual);
  double stdDevError = calcStandardDeviation(errorResiduals, meanError);

  // Calculate max values
  double maxRotationResidual = *std::max_element(rotationResiduals.begin(), rotationResiduals.end());
  double maxClockResidual = *std::max_element(clockResiduals.begin(), clockResiduals.end());
  double maxParallelResidual = *std::max_element(parallelResiduals.begin(), parallelResiduals.end());
  double maxTranslationResidual = *std::max_element(translationResiduals.begin(), translationResiduals.end());
  double maxError = *std::max_element(errorResiduals.begin(), errorResiduals.end());

  // Output results
  std::cout << "Mean residual fMo(" << inPoses.size() << ") - rotation (deg) = " << meanRotationResidual << std::endl;
  std::cout << "Mean residual fMo(" << inPoses.size() << ") - clockwise rotation (deg) = " << meanClockResidual << std::endl;
  std::cout << "Mean residual fMo(" << inPoses.size() << ") - parallel rotation (deg) = " << meanParallelResidual << std::endl;
  std::cout << "Mean residual fMo(" << inPoses.size() << ") - translation (m) = " << meanTranslationResidual << std::endl;
  std::cout << "Mean residual fMo(" << inPoses.size() << ") - error (m) = " << meanError << std::endl;

  std::cout << "Standard deviation fMo(" << inPoses.size() << ") - rotation (deg) = " << stdDevRotation << std::endl;
  std::cout << "Standard deviation fMo(" << inPoses.size() << ") - clockwise rotation (deg) = " << stdDevClock << std::endl;
  std::cout << "Standard deviation fMo(" << inPoses.size() << ") - parallel rotation (deg) = " << stdDevParallel << std::endl;
  std::cout << "Standard deviation fMo(" << inPoses.size() << ") - translation (m) = " << stdDevTranslation << std::endl;
  std::cout << "Standard deviation fMo(" << inPoses.size() << ") - error (m) = " << stdDevError << std::endl;

  std::cout << "Max residual fMo(" << inPoses.size() << ") - rotation (deg) = " << maxRotationResidual << std::endl;
  std::cout << "Max residual fMo(" << inPoses.size() << ") - clockwise rotation (deg) = " << maxClockResidual << std::endl;
  std::cout << "Max residual fMo(" << inPoses.size() << ") - parallel rotation (deg) = " << maxParallelResidual << std::endl;
  std::cout << "Max residual fMo(" << inPoses.size() << ") - translation (m) = " << maxTranslationResidual << std::endl;
  std::cout << "Max residual fMo(" << inPoses.size() << ") - error (m) = " << maxError << std::endl;

  return avgPose;
}

