#include <chrono>
#include <string>
#include <fstream>
#include <iomanip>

#include <glog/logging.h>

#include "sli_slam/VisualOdometry.hpp"
#include "sli_slam/Config.hpp"

using std::string;
using std::ofstream;
using std::endl;
using std::fixed;
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

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetLeftCam(dataset_->GetCameraById(0));
    frontend_->SetRightCam(dataset_->GetCameraById(1));

    backend_->SetMap(map_);
    backend_->SetLeftCam(dataset_->GetCameraById(0));
    backend_->SetRightCam(dataset_->GetCameraById(1));

    enable_viewer_ = Config::Get<int>("enable_viewer");
    if(enable_viewer_){
        viewer_ = Viewer::Ptr(new Viewer);
        viewer_->SetMap(map_);
    }

    record_trajectory_ = Config::Get<int>("save_trajectory");
    if(record_trajectory_){
        out_file_ = ofstream("EstimatedTrajectory.txt");
        if(!out_file_.is_open()){
            return false;
        }
    }

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

    if(record_trajectory_){
        SaveTrajectory(frontend_->GetCurrentPose());
    }
    
    return if_success;
}

bool VisualOdometry::SaveTrajectory(Sophus::SE3d pose){
    Mat34 m = pose.matrix3x4();
    out_file_ << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " " << m(0, 3) << " "
              << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << " " << m(1, 3) << " "
              << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " " << m(2, 3) << endl;

    return true;
}