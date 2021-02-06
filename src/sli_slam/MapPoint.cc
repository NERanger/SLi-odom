#include <mutex>

#include "sli_slam/MapPoint.hpp"
#include "sli_slam/Feature.hpp"

using std::mutex;
using std::lock_guard;

using sli_slam::MapPoint;
using sli_slam::Feature;

Vec3 MapPoint::Position(){
    lock_guard<mutex> lck(data_mutex_);
    return position_;
}

void MapPoint::SetPosition(const Vec3 &position){
    lock_guard<mutex> lck(data_mutex_);
    position_ = position;
}

void MapPoint::AddObservation(Feature::Ptr feat){
    lock_guard<mutex> lck(data_mutex_);
    observations_.push_back(feat);
    ++observed_times_;
}

void MapPoint::RemoveObservation(Feature::Ptr feat){
    lock_guard<mutex> lck(data_mutex_);
    for(auto iter = observations_.begin(); iter != observations_.end(); iter++){
        if(iter->lock() == feat){
            observations_.erase(iter);
            feat->RelatedMapPoint().reset();
            observed_times_ -= 1;
            break;
        }
    }
}

MapPoint::Ptr MapPoint::CreateNewMapPoint(){
    static unsigned long factory_id = 0;
    
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id;
    factory_id += 1;

    return new_mappoint;
}