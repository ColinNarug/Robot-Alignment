#ifndef __markley__
#define __markley__
#include <visp3/vision/vpPose.h>

// Markley algorithm for optimal pose estimation
// TODO add weights
vpHomogeneousMatrix optimalPose(std::vector<vpHomogeneousMatrix> inputPoses);




#endif /* __markley__ */