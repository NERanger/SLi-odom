#ifndef SLI_SLAM_BACKEND_HPP
#define SLI_SLAM_BACKEND_HPP

#include <memory>

#include "sli_slam/common.hpp"

namespace sli_slam{

class Backend{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;
};

} // namespace sli_slam

#endif