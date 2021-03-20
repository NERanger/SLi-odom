#ifndef SLI_SLAM_LIDAR_HPP
#define SLI_SLAM_LIDAR_HPP

#include <memory>

#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"

namespace sli_slam{

class Lidar{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Lidar> Ptr;

    Lidar() = default;
    Lidar(const Sophus::SE3d &pose){pose_ = pose;}

private:
    // T_c_li
    // Tranformation form LiDAR to left camera
    Sophus::SE3d pose_;
};

}

#endif