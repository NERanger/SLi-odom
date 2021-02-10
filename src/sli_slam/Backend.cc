#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <map>

#include <glog/logging.h>

#include "sli_slam/Backend.hpp"
#include "sli_slam/G2oTypes.hpp"
#include "sli_slam/Algorithm.hpp"

using std::thread;
using std::bind;
using std::mutex;
using std::lock_guard;
using std::unique_lock;
using std::shared_ptr;
using std::map;

using Sophus::SE3d;

using g2o::make_unique;
using g2o::BlockSolver_6_3;
using g2o::LinearSolverCSparse;
using g2o::SparseOptimizer;
using g2o::OptimizationAlgorithmLevenberg;

using sli_slam::Backend;
using sli_slam::Frame;
using sli_slam::Feature;
using sli_slam::Map;
using sli_slam::ToVec2;

Backend::Backend(){
    backend_running_.store(true);
    backend_thread_ = thread(bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap(){
    lock_guard<mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop(){
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop(){
    while(backend_running_.load()){
        unique_lock<mutex> lock(data_mutex_);
        map_update_.wait(lock);

        // Backend only optimize active frames and landmarks
        Map::KeyFramesType active_kfs = map_->GetAllActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetAllActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void Backend::Optimize(Map::KeyFramesType& keyframes, Map::LandmarksType& landmarks){
    // Setup g2o
    typedef BlockSolver_6_3 BlockSolverType;
    typedef LinearSolverCSparse<BlockSolverType::PoseMatrixType>
            LinearSolverType;

    auto solver = new OptimizationAlgorithmLevenberg(
                        make_unique<BlockSolverType>(
                            make_unique<LinearSolverType>()));

    SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Pose vertex, using keyframe id
    map<unsigned long, VertexPose *> vertices_pose;
    unsigned long max_kf_id = 0;

    for(auto &keyframe : keyframes){
        Frame::Ptr kf = keyframe.second;

        // Raw pointer
        // TODO: replace with smart pointer
        // std::make_shared<VertexPose>();
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(kf->KeyFrameId());
        vertex_pose->setEstimate(kf->Pose());

        optimizer.addVertex(vertex_pose);
        if(kf->KeyFrameId() > max_kf_id){
            max_kf_id = kf->KeyFrameId();
        }

        // Camera pose vertex
        vertices_pose.insert({kf->KeyFrameId(), vertex_pose});
    }

    map<unsigned long, VertexXYZ *> vertices_landmarks;

    // Intrinsics & extrinsics
    Mat33 K = cam_left_->GetIntrinsicMatrix();
    SE3d left_ext = cam_left_->Pose();
    SE3d right_ext = cam_right_->Pose();
    
    // Edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel threshold
    map<EdgeProjection *, Feature::Ptr> edges_feature;

    for(auto &landmark : landmarks){
        if(landmark.second->IsOutlier()){
            continue;
        }

        unsigned long landmark_id = landmark.second->Id();
        auto observations = landmark.second->Observations();

        for(auto &obs : observations){
            Feature::Ptr feat = obs.lock();
            if(feat == nullptr){
                continue;
            }

            Frame::Ptr frame = feat->RelatedFrame().lock();
            if(feat->IsOutlier() || frame == nullptr){
                continue;
            }

            // Raw pointer
            // TODO: replace with smart pointer
            // std::make_shared<EdgeProjection>();
            EdgeProjection *edge = nullptr;
            if (feat->IsOnLeftImg()) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // If landmark is not included in optimizer yet,
            // create a new vertex
            if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end()){
                VertexXYZ *v = new VertexXYZ();
                v->setEstimate(landmark.second->Position());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);  // ???
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0, vertices_pose.at(frame->KeyFrameId()));  // Pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id)); // Landmark
            edge->setMeasurement(ToVec2(feat->Position().pt));
            edge->setInformation(Mat22::Identity());

            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th); // ???
            edge->setRobustKernel(rk);
            edges_feature.insert({edge, feat});

            optimizer.addEdge(edge);

            index += 1;
        }
    }

    // Optimize and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0;
    int cnt_inlier = 0;
    int iteration = 0;

    while(iteration < 5){
        cnt_outlier = 0;
        cnt_inlier = 0;

        // Determine if we want to adjust the outlier threshold
        for (auto &ef : edges_feature) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }

        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration += 1;
        }
    }

    for(auto &ef : edges_feature){
        if(ef.first->chi2() > chi2_th){
            ef.second->SetIsOutlier(true);
            // Remove the observation
            ef.second->
                RelatedMapPoint().lock()->RemoveObservation(ef.second);
        }else{
            ef.second->SetIsOutlier(false);
        }
    }

    LOG(INFO) << "Outlier/inlier in optimization: " 
              << cnt_outlier << "/" << cnt_inlier;

    // Set pose and landmark position
    for(auto &v : vertices_pose){
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for(auto &v : vertices_landmarks){
        landmarks.at(v.first)->SetPosition(v.second->estimate());
    }
}