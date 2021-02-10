#ifndef SLI_SLAM_VIEWER_HPP
#define SLI_SLAM_VIEWER_HPP

#include <unordered_map>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>

#include "sli_slam/Common.hpp"
#include "sli_slam/Map.hpp"
#include "sli_slam/Frame.hpp"

namespace sli_slam{

// Data visualization
class Viewer{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    // Customized contructor
    // Launch visualization thread and pending
    Viewer();

    void SetMap(Map::Ptr map){map_ = map;}
    void AddCurrentFrame(Frame::Ptr current_frame);
    void UpdateMap();

    void Close();

private:
    // Viewer thread main function
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float* color);
    void DrawMapPoints();
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    // Plot the features in current frame into an image
    cv::Mat PlotFeatureOnImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    // ??? wtf
    // Redundant data?
    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};

} // namespace sli_slam

#endif