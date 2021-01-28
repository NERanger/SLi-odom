#include "stereo_slam/Feature.hpp"
#include "stereo_slam/MapPoint.hpp"

using stereo_slam::Feature;
using stereo_slam::MapPoint;

std::weak_ptr<MapPoint> Feature::RelatedMapPoint(){
    return map_point_;
}