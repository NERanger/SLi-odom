#ifndef STEREO_SLAM_FRAME_HPP
#define STEREO_SLAM_FRAME_HPP

#include <memory>
#include <mutex>
#include <vector>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include "stereo_slam/Feature.hpp"

namespace stereo_slam{

class Feature;

class Frame{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Frame> Ptr;

    Frame() = default;
    Frame(long id, double time_stamp, const Sophus::SE3d &pose, 
          const cv::Mat &left_img, const cv::Mat &right_img);

    Sophus::SE3d Pose();

    void SetPose(const Sophus::SE3d &pose);
    void SetKeyFrame();

    unsigned long KeyFrameId();
    std::vector<Feature::Ptr> FeatureLeft();
    std::vector<Feature::Ptr> FeatureRight();

    static Ptr CreateFrame();

private:
    unsigned long id_;           // Frame id
    unsigned long keyframe_id_;
    bool is_key_frame_;
    double time_stamp_;          // Time stamp

    Sophus::SE3d pose_;          // Frame pose in Tcw

    std::mutex pose_mutex_;

    cv::Mat left_img_, right_img_;

    // Feature in left image
    std::vector<Feature::Ptr> feature_left_;

    // Feature in right image
    std::vector<Feature::Ptr> feature_right_;
};

} // namespace stereo_slam

#endif