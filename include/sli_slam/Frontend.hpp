#ifndef SLI_SLAM_FRONTEND_HPP
#define SLI_SLAM_FRONTEND_HPP

#include <memory>

#include <Eigen/Core>

#include "sli_slam/Common.hpp"
#include "sli_slam/Map.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Camera.hpp"
#include "sli_slam/Backend.hpp"
#include "sli_slam/Viewer.hpp"

namespace sli_slam{

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

    // Customized contructor
    Frontend();

    // Add a frame and compute pose
    bool AddFrame(Frame::Ptr frame);

    void SetMap(Map::Ptr map) {map_ = map;}
    void SetBackend(Backend::Ptr backend) {backend_ = backend;}
    void SetViewer(Viewer::Ptr viewer) {viewer_ = viewer;}
    void SetLeftCam(Camera::Ptr left_cam) {camera_left_ = left_cam;}
    void SetRightCam(Camera::Ptr right_cam) {camera_right_ = right_cam;}

    FrontendStatus GetStatus() const {return status_;}

    Sophus::SE3d GetCurrentPose();

private:
    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * Estimate current frame's pose
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * Set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
    int FindFeaturesInRight();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
    int TriangulateNewPoints();

    /**
     * Set the features in keyframe as new observation of the map points
     */
    void SetObservationsForKeyFrame();

    FrontendStatus status_ = FrontendStatus::kInitiating;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    // Relative motion between current frame and last frame
    // Used as initial value for current frame pose
    Sophus::SE3d relative_motion_;

    // Used for testing the frame is keyframe or not
    int tracking_inliers_ = 0;

    // Params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_detector_;  // feature detector in opencv
};

} // namespace sli_slam

#endif