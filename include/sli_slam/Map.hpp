#ifndef SLI_SLAM_MAP_HPP
#define SLI_SLAM_MAP_HPP

#include <memory>
#include <mutex>
#include <unordered_map>

#include "sli_slam/common.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/MapPoint.hpp"

namespace sli_slam{

class Map{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFramesType;

    Map() = default;

    void InsertKeyFrame(Frame::Ptr frame);
    void InsertMapPoint(MapPoint::Ptr map_point);
    LandmarksType GetAllMapPoints();
    KeyFramesType GetAllKeyFrames();
    LandmarksType GetAllActiveMapPoints();
    KeyFramesType GetAllActiveKeyFrames();

    // Clean landmarks with 0 observation time
    void CleanMap();

private:
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyFramesType keyframes_;
    KeyFramesType active_keyframes_;

    Frame::Ptr current_frame_ = nullptr;

    // Setting
    int num_active_keyframes_ = 7; // Number of active keyframe
};

} // namespace sli_slam

#endif