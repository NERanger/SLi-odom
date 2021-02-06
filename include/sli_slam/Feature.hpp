#ifndef SLI_SLAM_FEATURE_HPP
#define SLI_SLAM_FEATURE_HPP

#include <memory>

#include <Eigen/Core>

#include <opencv2/features2d.hpp>

namespace sli_slam{

class Frame;
class MapPoint;

class Feature{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef std::shared_ptr<Feature> Ptr;

    Feature() = default;

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp) : frame_(frame), position_(kp){}

    std::weak_ptr<MapPoint> RelatedMapPoint() const {return map_point_;}
    cv::KeyPoint Position() const {return position_;}
    bool IsOutlier() const {return is_outlier_;}
    bool IsOnLeftImg() const {return is_on_left_img_;}

    void SetRelatedMapPoint(MapPoint::Ptr map_point) {map_point_ = map_point;}
    void SetIsOutlier(bool is_outlier) {is_outlier_ = is_outlier;}
    void SetIsOnLeftImg(bool is_on_left_img) {is_on_left_img_ = is_on_left_img;}
private:
    std::weak_ptr<Frame> frame_;         // Frame with this feature
    std::weak_ptr<MapPoint> map_point_;  // Related map point

    cv::KeyPoint position_;          // keypoint position

    bool is_outlier_ = false;
    bool is_on_left_img_ = true;         // False if on right image

};

} // namespace sli_slam

#endif