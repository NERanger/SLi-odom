#ifndef SLI_SLAM_ALGORITHM_HPP
#define SLI_SLAM_ALGORITHM_HPP

#include <opencv2/opencv.hpp>

#include "sli_slam/Common.hpp"

namespace sli_slam{

inline Vec2 ToVec2(const cv::Point2f p) {return Vec2(p.x, p.y);}

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
bool Triangulation(const std::vector<Sophus::SE3d> &poses, 
                   const std::vector<Vec3> points, 
                   Vec3 &pt_world);

} // namespace sli_slam

#endif