#ifndef SLI_SLAM_FRONTEND_HPP
#define SLI_SLAM_FRONTEND_HPP

#include "sli_slam/common.hpp"

namespace sli_slam{

enum class FrontendStatus{
    kInitiating,
    kTrackingGood,
    kTrackingBad,
    kTrackingLost
};

class Frontend {
public:
    
};

} // namespace sli_slam

#endif