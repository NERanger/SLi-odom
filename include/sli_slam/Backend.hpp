#ifndef SLI_SLAM_BACKEND_HPP
#define SLI_SLAM_BACKEND_HPP

#include <memory>
#include <thread>
#include <condition_variable>
#include <atomic>

#include "sli_slam/Common.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Map.hpp"
#include "sli_slam/Camera.hpp"

namespace sli_slam{

// Backend runs on a separated thread
// Execute optimization when update map
// Map update is triggered by frontend
class Backend{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    // Customized contructor
    // Launch optimization thread and pending
    Backend();

    // Set left and right camera, used for obtain intrinsics & extrinsics
    void SetCamera(Camera::Ptr left_cam, Camera::Ptr right_cam);
    void SetMap(Map::Ptr map);
    
    // Trigger map update and start optimization
    void UpdateMap();

    // Turn down backend optimization
    void Stop();

private:
    // Backend thread main function
    void BackendLoop();

    // Optimize given keyframe and map point
    void Optimize(Map::KeyFramesType& keyframes, Map::LandmarksType& landmarks);

    Map::Ptr map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr;
    Camera::Ptr cam_right_ = nullptr;

};

} // namespace sli_slam

#endif