#include <chrono>

#include <glog/logging.h>

#include "sli_slam/VisualOdometry.hpp"
#include "sli_slam/Config.hpp"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

using sli_slam::VisualOdometry;
using sli_slam::Config;

bool VisualOdometry::Init(){
    // Read from config file
    if(Config::SetParameterFile(config_file_path_) == false){
        return false;
    }

    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // Create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetLeftCam(dataset_->GetCameraById(0));
    frontend_->SetRightCam(dataset_->GetCameraById(1));

    backend_->SetMap(map_);
    backend_->SetLeftCam(dataset_->GetCameraById(0));
    backend_->SetRightCam(dataset_->GetCameraById(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run(){
    while(true){
        LOG(INFO) << "VO running";
        if(Step() == false){
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step(){
    Frame::Ptr new_frame = dataset_->NextFrame();
    if(new_frame == nullptr){
        return false;
    }

    steady_clock::time_point t1 = steady_clock::now();
    bool if_success = frontend_->AddFrame(new_frame);
    steady_clock::time_point t2 = steady_clock::now();
    milliseconds time_used = duration_cast<milliseconds>(t2 - t1);
    LOG(INFO) << "VO time cost: " << time_used.count() << " milliseconds.";
    
    return if_success;
}