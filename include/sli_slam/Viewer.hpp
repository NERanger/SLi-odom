#ifndef SLI_SLAM_VIEWER_HPP
#define SLI_SLAM_VIEWER_HPP

#include <memory>

#include "sli_slam/common.hpp"

namespace sli_slam{

class Viewer{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;
};

} // namespace sli_slam

#endif