#include "sli_slam/Feature.hpp"
#include "sli_slam/MapPoint.hpp"

using sli_slam::Feature;
using sli_slam::MapPoint;

std::weak_ptr<MapPoint> Feature::RelatedMapPoint(){
    return map_point_;
}