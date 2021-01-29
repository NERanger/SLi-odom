#ifndef SLI_SLAM_CAMERA_HPP
#define SLI_SLAM_CAMERA_HPP

#include <memory>

#include "sli_slam/common.hpp"

namespace sli_slam{

class Camera{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;
};

} // namespace sli_slam

#endif