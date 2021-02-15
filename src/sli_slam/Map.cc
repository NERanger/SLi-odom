#include <utility>
#include <limits>
#include <mutex>

#include <glog/logging.h>

#include "sli_slam/Map.hpp"
#include "sli_slam/Feature.hpp"

using std::make_pair;
using std::numeric_limits;
using std::mutex;
using std::lock_guard;

using Sophus::SE3d;

using sli_slam::Map;

void Map::InsertKeyFrame(Frame::Ptr frame){
    current_frame_ = frame;
    if(keyframes_.find(frame->KeyFrameId()) == keyframes_.end()){
        keyframes_.insert(make_pair(frame->KeyFrameId(), frame));
        active_keyframes_.insert(make_pair(frame->KeyFrameId(), frame));
    } else {
        keyframes_[frame->KeyFrameId()] = frame;
        active_keyframes_[frame->KeyFrameId()] = frame;
    }

    if(active_keyframes_.size() > num_active_keyframes_){
        RemoveOldKeyframe();
    }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point){
    if(landmarks_.find(map_point->Id()) == landmarks_.end()){
        landmarks_.insert(make_pair(map_point->Id(), map_point));
        active_landmarks_.insert(make_pair(map_point->Id(), map_point));
    } else {
        landmarks_[map_point->Id()] = map_point;
        active_landmarks_[map_point->Id()] = map_point;
    }
}

Map::LandmarksType Map::GetAllMapPoints(){
    lock_guard<mutex> lck(data_mutex_);
    return landmarks_;
}

Map::KeyFramesType Map::GetAllKeyFrames(){
    lock_guard<mutex> lck(data_mutex_);
    return keyframes_;
}

Map::LandmarksType Map::GetAllActiveMapPoints(){
    lock_guard<mutex> lck(data_mutex_);
    return active_landmarks_;
}

Map::KeyFramesType Map::GetAllActiveKeyFrames(){
    lock_guard<mutex> lck(data_mutex_);
    return active_keyframes_;
}

void Map::RemoveOldKeyframe(){
    if(current_frame_ == nullptr) return;

    // Find the furthest and closest keyframe
    // in position distance

    double max_dis = numeric_limits<double>::min();
    double min_dis = numeric_limits<double>::max();

    double max_keyframe_id = 0;
    double min_keyframe_id = 0;

    SE3d Twc = current_frame_->Pose().inverse();
    for (auto &kf : active_keyframes_){
        if(kf.second == current_frame_){
            continue;
        }

        double dis = (kf.second->Pose() * Twc).log().norm();

        if(dis > max_dis){
            max_dis = dis;
            max_keyframe_id = kf.first;
        }
        if(dis < min_dis){
            min_dis = dis;
            min_keyframe_id = kf.first;
        }
    }

    // Closest keyframe threshold (meter)
    const double min_dis_thresh = 0.2;

    Frame::Ptr frame_to_remove = nullptr;
    if(min_dis < min_dis_thresh){
        // If the closest keyframe distance is lower than threshold
        // Remove the closest keyframe first
        frame_to_remove = keyframes_.at(min_keyframe_id);
    } else {
        // Remove the furthest keyframe
        frame_to_remove = keyframes_.at(max_keyframe_id);
    }

    LOG(INFO) << "Remove keyframe " << frame_to_remove->KeyFrameId();

    // Remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->KeyFrameId());
    for(auto feat : frame_to_remove->FeatureLeft()){
        MapPoint::Ptr mp = feat->RelatedMapPoint().lock();
        if(mp){
            mp->RemoveObservation(feat);
        }
    }

    for(auto feat : frame_to_remove->FeatureRight()){
        if(feat == nullptr){
            continue;
        }

        MapPoint::Ptr mp = feat->RelatedMapPoint().lock();
        if(mp){
            mp->RemoveObservation(feat);
        }else{
        }
    }

    CleanMap();
}

void Map::CleanMap(){
    int cnt_landmark_removed = 0;
    for(auto iter = active_landmarks_.begin(); 
        iter != active_landmarks_.end();){
        
        if(iter->second->ObservedTimes() == 0){
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed += 1;
        } else {
            ++iter;
        }
    }

    LOG(INFO) << "Removed " << cnt_landmark_removed << "active landmarks";
}