#include <vector>

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include "sli_slam/Frontend.hpp"
#include "sli_slam/Config.hpp"
#include "sli_slam/G2oTypes.hpp"
#include "sli_slam/Map.hpp"
#include "sli_slam/Algorithm.hpp"

using std::vector;

using cv::Mat;
using cv::Size;
using cv::KeyPoint;
using cv::Point2f;
using cv::GFTTDetector;
using cv::TermCriteria;
using cv::calcOpticalFlowPyrLK;

using Eigen::Matrix2d;

using g2o::BlockSolver_6_3;
using g2o::LinearSolverDense;
using g2o::make_unique;
using g2o::SparseOptimizer;
using g2o::RobustKernelHuber;
using g2o::OptimizationAlgorithmLevenberg;

using sli_slam::Config;
using sli_slam::Frame;
using sli_slam::FrontendStatus;
using sli_slam::Frontend;
using sli_slam::MapPoint;
using sli_slam::ToVec2;

Frontend::Frontend(){
    gftt_detector_ = 
        GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);

    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(Frame::Ptr frame){
    current_frame_ = frame;

    switch(status_){
        case FrontendStatus::kInitiating:
            StereoInit();
            break;
        case FrontendStatus::kTrackingGood:
            // TODO
        case FrontendStatus::kTrackingBad:
            Track();
            break;
        case FrontendStatus::kTrackingLost:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track(){
    if(last_frame_){
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::kTrackingGood;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::kTrackingBad;
    } else {
        // lost
        status_ = FrontendStatus::kTrackingLost;
    }
}

int Frontend::TrackLastFrame(){
    // Use Lucas-Kanade optical flow
    // to estimate pixel motion in right image

    vector<Point2f> kps_last, kps_current; // Key point

    for(auto &kp : last_frame_->FeatureLeft()){
        MapPoint::Ptr mp = kp->RelatedMapPoint().lock();
        if(mp){
            Vec2 px = camera_left_->World2Pixel(mp->Position(), current_frame_->Pose());
            kps_last.emplace_back(kp->Position().pt);
            kps_current.emplace_back(Point2f(px[0], px[1]));
        } else {
            kps_last.emplace_back(kp->Position().pt);
            kps_current.emplace_back(kp->Position().pt);
        }
    }

    vector<uchar> status;
    Mat error;
    calcOpticalFlowPyrLK(
        last_frame_->LeftImg(), current_frame_->LeftImg(), kps_last, 
        kps_current, status, error, Size(11, 11), 3, 
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for(size_t i = 0; i < status.size(); ++i){
        if(status[i]){
            KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            current_frame_->FeatureLeft().emplace_back(feature);
            num_good_pts += 1;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";

    return num_good_pts;

}

int Frontend::EstimateCurrentPose(){
    // Setup g2o
    typedef BlockSolver_6_3 BlockSolverType;
    typedef LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new OptimizationAlgorithmLevenberg(
        make_unique<BlockSolverType>(
            make_unique<LinearSolverType>()));

    SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Vertex
    VertexPose *vertex_pose = new VertexPose();  // Camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->GetIntrinsicMatrix();

    // Edges
    int index = 1;
    vector<EdgeProjectionPoseOnly *> edges;
    vector<Feature::Ptr> features;
    for(size_t i = 0; i < current_frame_->FeatureLeft().size(); ++i){
        MapPoint::Ptr mp = current_frame_->FeatureLeft()[i]->RelatedMapPoint().lock();
        if(mp){
            features.emplace_back(current_frame_->FeatureLeft()[i]);

            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->Position(), K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(ToVec2(current_frame_->FeatureLeft()[i]->Position().pt));
            edge->setInformation(Matrix2d::Identity());
            edge->setRobustKernel(new RobustKernelHuber);
            edges.emplace_back(edge);
            optimizer.addEdge(edge);
            index += 1;
        }
    }

    // Estimate pose, then determine if is outlier
    const double chi2_thresh = 5.991;
    int cnt_outlier = 0;
    for(int iter = 0; iter < 4; ++iter){
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // Count the outliers
        for(size_t i = 0; i < edges.size(); ++i){
            EdgeProjectionPoseOnly *e = edges[i];
            if(features[i]->IsOutlier()){
                e->computeError();
            }
            if(e->chi2() > chi2_thresh){
                features[i]->SetIsOutlier(true);
                e->setLevel(1);
                cnt_outlier++;
            }else{
                features[i]->SetIsOutlier(false);
                e->setLevel(0);
            }

            if(iter == 2){
                // ???
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;

    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->IsOutlier()) {
            feat->RelatedMapPoint().reset();
            feat->SetIsOutlier(false);  // maybe we can still use it in future
        }
    }

    return features.size() - cnt_outlier;
}
