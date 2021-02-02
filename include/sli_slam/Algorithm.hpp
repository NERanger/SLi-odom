#ifndef SLI_SLAM_ALGORITHM_HPP
#define SLI_SLAM_ALGORITHM_HPP

#include <opencv2/opencv.hpp>

#include "sli_slam/Common.hpp"

namespace sli_slam{

inline Vec2 ToVec2(const cv::Point2f p){return Vec2(p.x, p.y);}

}


#endif