#include <vector>

#include "sli_slam/Frame.hpp"

using std::vector;
using std::mutex;
using std::lock_guard;

using Sophus::SE3d;

using sli_slam::Frame;
using sli_slam::Feature;

SE3d Frame::Pose(){
    lock_guard<mutex> lck(pose_mutex_);
    return pose_;
}

void Frame::SetPose(const SE3d &pose) {
    lock_guard<mutex> lck(pose_mutex_);
    pose_ = pose;
}

Frame::Ptr Frame::CreateFrame(){
    static unsigned long factory_id = 0;
    
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id;
    factory_id += 1;

    return new_frame;
}

void Frame::SetKeyFrame(){
    static unsigned long keyframe_factory_id = 0;
    is_key_frame_ = true;
    keyframe_id_ = keyframe_factory_id;
    keyframe_factory_id += 1;
}
