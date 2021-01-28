#ifndef STEREO_SLAM_FEATURE_HPP
#define STEREO_SLAM_FEATURE_HPP

#include <memory>

#include <Eigen/Core>

#include <opencv2/features2d.hpp>

namespace stereo_slam{

class Frame;
class MapPoint;

class Feature{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef std::shared_ptr<Feature> Ptr;

    Feature() = default;

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp) : frame_(frame), keypoint_pos_(kp){}

    std::weak_ptr<MapPoint> RelatedMapPoint();
private:
    std::weak_ptr<Frame> frame_;         // Frame with this feature
    std::weak_ptr<MapPoint> map_point_;  // Related map point

    cv::KeyPoint keypoint_pos_;          // keypoint position

    bool is_outlier_ = false;
    bool is_on_left_img_ = true;         // False if on right image


};

} // namespace stereo_slam

#endif