#include <vector>

#include "stereo_slam/Frame.hpp"

using std::vector;
using std::mutex;
using std::lock_guard;

using Sophus::SE3d;

using stereo_slam::Frame;
using stereo_slam::Feature;

SE3d Frame::Pose(){
    lock_guard<mutex> lck(pose_mutex_);
    return pose_;
}

void Frame::SetPose(const SE3d &pose){
    lock_guard<mutex> lck(pose_mutex_);
    pose_ = pose;
}

unsigned long Frame::KeyFrameId(){
    return keyframe_id_;
}

vector<Feature::Ptr> Frame::FeatureLeft(){
    return feature_left_;
}

vector<Feature::Ptr> Frame::FeatureRight(){
    return feature_right_;
}

Frame::Ptr Frame::CreateFrame(){
    static unsigned long factory_id = 0;
    
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id;
    factory_id += 1;

    return new_frame;
}
