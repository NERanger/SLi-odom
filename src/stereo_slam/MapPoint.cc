#include <mutex>

#include "stereo_slam/MapPoint.hpp"
#include "stereo_slam/Feature.hpp"

using std::mutex;
using std::lock_guard;

using stereo_slam::MapPoint;
using stereo_slam::Feature;

Vec3 MapPoint::Pos(){
    lock_guard<mutex> lck(data_mutex_);
    return pos_;
}

unsigned long MapPoint::Id(){
    return id_;
}

void MapPoint::SetPos(const Vec3 &pos){
    lock_guard<mutex> lck(data_mutex_);
    pos_ = pos;
}

int MapPoint::ObservedTimes(){
    return observed_times_;
}

void MapPoint::AddObservation(Feature::Ptr feat){
    lock_guard<mutex> lck(data_mutex_);
    observations_.emplace_back(feat);
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