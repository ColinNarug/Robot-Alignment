#ifndef __kabsch__
#define __kabsch__
#include <visp3/vision/vpPose.h>

// Kabsh algorithm for camera pose estimation
// Assume that the tracked coordinates and model coordinates have same scale
vpHomogeneousMatrix findPose(std::vector<vpColVector> inputPoints, std::vector<vpColVector> outputPoints);




#endif /* __kabsch__ */