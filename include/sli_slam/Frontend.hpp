#ifndef SLI_SLAM_FRONTEND_HPP
#define SLI_SLAM_FRONTEND_HPP

#include <memory>

#include <Eigen/Core>

#include "sli_slam/common.hpp"
#include "sli_slam/Map.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Backend.hpp"
#include "sli_slam/Viewer.hpp"
#include "sli_slam/Camera.hpp"

namespace sli_slam{

class Backend;
class Viewer;
class Frame;
class Camera;
class Map;

enum class FrontendStatus{
    kInitiating,
    kTrackingGood,
    kTrackingBad,
    kTrackingLost
};

// Frontend will estimation current pose, add keyframe
// and trigger optimization
class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend() = default;

    bool AddFrame(Frame::Ptr frame);
    void SetMap(Map::Ptr map);
    void SetBackend(Backend::Ptr backend);
    void SetViewer(Viewer::Ptr viewer);
    void SetCameras(Camera::Ptr left, Camera::Ptr right);

    FrontendStatus GetStatus() const;

private:
    
};

} // namespace sli_slam

#endif