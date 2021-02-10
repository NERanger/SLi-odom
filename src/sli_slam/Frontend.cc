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
using cv::rectangle;
using cv::Point2f;
using cv::GFTTDetector;
using cv::TermCriteria;
using cv::calcOpticalFlowPyrLK;

using Eigen::Matrix2d;

using Sophus::SE3d;

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
using sli_slam::Triangulation;

Frontend::Frontend(){
    // Create GFTT detector
    // Param: maxCorners, qualityLevel, minDistance
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

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if(viewer_){
        viewer_->AddCurrentFrame(current_frame_);
    }

    return true;
}

int Frontend::TrackLastFrame(){
    // Use Lucas-Kanade optical flow
    // to estimate pixel motion in right image

    vector<Point2f> kps_last, kps_current; // Key point

    for(auto &kp : last_frame_->FeatureLeft()){
        MapPoint::Ptr mp = kp->RelatedMapPoint().lock();
        if(mp){
            Vec2 px = camera_left_->World2Pixel(mp->Position(), current_frame_->Pose());
            kps_last.push_back(kp->Position().pt);
            kps_current.push_back(Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->Position().pt);
            kps_current.push_back(kp->Position().pt);
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
            current_frame_->FeatureLeft().push_back(feature);
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
            features.push_back(current_frame_->FeatureLeft()[i]);

            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->Position(), K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(ToVec2(current_frame_->FeatureLeft()[i]->Position().pt));
            edge->setInformation(Matrix2d::Identity());
            edge->setRobustKernel(new RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index += 1;
        }
    }

    // Estimate pose, then determine if it is outlier
    // We perform 4 optimizations, 
    // after each optimization we classify observation as inlier/outlier.
    // At the next optimization, outliers are not included, 
    // but at the end they can be classified as inliers again.
    // Refference: https://github.com/RainerKuemmerle/g2o/issues/259
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
                // After 2 iterations the robust kernel is removed
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
            // feat->SetIsOutlier(false);  // maybe we can still use it in future
        }
    }

    return features.size() - cnt_outlier;
}


bool Frontend::InsertKeyframe(){
    if(tracking_inliers_ >= num_features_needed_for_keyframe_){
        // Still have enough features, don't insert keyframe
        return false;
    }

    // Set current frame to keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->FrameId() << " as keyframe "
              << current_frame_->KeyFrameId();

    SetObservationsForKeyFrame();
    DetectFeatures();  // Detect new features

    // Find feature match in right camera image
    FindFeaturesInRight();
    // Perform triangulation to get depth
    TriangulateNewPoints();
    // Update backends since we have a new keyframe
    backend_->UpdateMap();

    if(viewer_){
        viewer_->UpdateMap();
    }

    return true;
}

void Frontend::SetObservationsForKeyFrame(){
    for(auto &feat : current_frame_->FeatureLeft()){
        MapPoint::Ptr mp = feat->RelatedMapPoint().lock();
        if(mp){
            mp->AddObservation(feat);
        }
    }
}

int Frontend::DetectFeatures(){
    Mat mask(current_frame_->LeftImg().size(), CV_8UC1, 255);
    for(auto &feat : current_frame_->FeatureLeft()){
        // Since feature min distance is set to 20,
        // we do not need to look for features in the 20x20 
        // nighboring region of current features
        rectangle(mask, feat->Position().pt - Point2f(10, 10),
                  feat->Position().pt + Point2f(10, 10), 0, CV_FILLED);
    }

    vector<KeyPoint> keypoints;
    gftt_detector_->detect(current_frame_->LeftImg(), keypoints, mask);
    int cnt_deteted = 0;
    for(auto &kp : keypoints){
        current_frame_->FeatureLeft().push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_deteted += 1;
    }

    LOG(INFO) << "Detect " << cnt_deteted << " new features";
    return cnt_deteted;
}

int Frontend::FindFeaturesInRight(){
    // Use LK optical flow to estimate points in the right image
    vector<Point2f> kps_left, kps_right;
    LOG(INFO) << "Left feat size " << current_frame_->FeatureLeft().size();
    for(auto &kp : current_frame_->FeatureLeft()){
        kps_left.push_back(kp->Position().pt);
        auto mp = kp->RelatedMapPoint().lock();
        if(mp){
            // Use projected points as initial guess if depth is availble
            auto px = 
                camera_right_->World2Pixel(mp->Position(), current_frame_->Pose());
            kps_right.push_back(Point2f(px[0], px[1]));
        }else{
            // Use same pixel in left image when depth is not availble
            kps_right.push_back(kp->Position().pt);
        }
    }

    vector<uchar> status;
    Mat error;
    LOG(INFO) << "Prepare Optical flow";
    LOG(INFO) << "LeftImg size " << current_frame_->LeftImg().size;
    LOG(INFO) << "RightImg size " << current_frame_->RightImg().size;
    LOG(INFO) << "kps_left size " << kps_left.size();
    LOG(INFO) << "kps_right size " << kps_right.size();
    calcOpticalFlowPyrLK(current_frame_->LeftImg(), current_frame_->RightImg(), 
        kps_left, kps_right, status, error, Size(11, 11), 3,
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    LOG(INFO) << "Optical flow done";

    int num_good_pts = 0;
    for(size_t i = 0; i < status.size(); ++i){
        if(status[i]){
            // Create new feature if match is found
            KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->SetIsOnLeftImg(false);
            current_frame_->FeatureRight().push_back(feat);
            num_good_pts += 1;
        }else{
            // Push back nullptr if no match is found
            current_frame_->FeatureRight().push_back(nullptr);
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

int Frontend::TriangulateNewPoints(){
    vector<SE3d> poses{camera_left_->Pose(), camera_right_->Pose()};
    SE3d current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->FeatureLeft().size(); ++i){
        if(current_frame_->FeatureLeft()[i]->RelatedMapPoint().expired() &&
           current_frame_->FeatureRight()[i] != nullptr){
            // Perform triangulation when
            // left feature has no related mappoint(i.e. depth is not available) and
            // right feature match exsits
            vector<Vec3> points{
                camera_left_->Pixel2Camera(
                    Vec2(current_frame_->FeatureLeft()[i]->Position().pt.x,
                         current_frame_->FeatureLeft()[i]->Position().pt.y)),
                camera_right_->Pixel2Camera(
                    Vec2(current_frame_->FeatureRight()[i]->Position().pt.x,
                         current_frame_->FeatureRight()[i]->Position().pt.y))};
            
            Vec3 pworld = Vec3::Zero();

            if(Triangulation(poses, points, pworld) && pworld[2] > 0){
                MapPoint::Ptr new_map_point = MapPoint::CreateNewMapPoint();

                // Current pworld is in stereo extrinsics coordinate
                // Still need transformation to world coordinate 
                pworld = current_pose_Twc * pworld;

                new_map_point->SetPosition(pworld);
                new_map_point->AddObservation(current_frame_->FeatureLeft()[i]);
                new_map_point->AddObservation(current_frame_->FeatureRight()[i]);
            
                current_frame_->FeatureLeft()[i]->SetRelatedMapPoint(new_map_point);
                current_frame_->FeatureRight()[i]->SetRelatedMapPoint(new_map_point);

                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts += 1;
            }
        }
    }

    LOG(INFO) << "New landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

bool Frontend::StereoInit(){
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if(num_coor_features < num_features_init_){
        return false;
    }

    bool build_map_success = BuildInitMap();
    if(build_map_success){
        status_ = FrontendStatus::kTrackingGood;
        if(viewer_){
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

bool Frontend::BuildInitMap(){
    vector<SE3d> poses{camera_left_->Pose(), camera_right_->Pose()};
    size_t cnt_init_landmarks = 0;
    for(size_t i = 0; i < current_frame_->FeatureLeft().size(); ++i){
        if(current_frame_->FeatureRight()[i] == nullptr){
            continue;
        }

        vector<Vec3> points{
            camera_left_->Pixel2Camera(
                Vec2(current_frame_->FeatureLeft()[i]->Position().pt.x,
                     current_frame_->FeatureLeft()[i]->Position().pt.y)),
            camera_right_->Pixel2Camera(
                Vec2(current_frame_->FeatureRight()[i]->Position().pt.x,
                     current_frame_->FeatureRight()[i]->Position().pt.y))};

        Vec3 pworld = Vec3::Zero();

        if(Triangulation(poses, points, pworld) && pworld[2] > 0){
            MapPoint::Ptr new_map_point = MapPoint::CreateNewMapPoint();
            
            // When init, extrinsics coordinate is the same with world coordinate
            new_map_point->SetPosition(pworld);
            new_map_point->AddObservation(current_frame_->FeatureLeft()[i]);
            new_map_point->AddObservation(current_frame_->FeatureRight()[i]);

            current_frame_->FeatureLeft()[i]->SetRelatedMapPoint(new_map_point);
            current_frame_->FeatureRight()[i]->SetRelatedMapPoint(new_map_point);

            cnt_init_landmarks += 1;

            map_->InsertMapPoint(new_map_point);
        }
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points.";

    return true;
}

bool Frontend::Reset(){
    // TODO
    LOG(INFO) << "NANI!? Reset is not implemented yet";
    return true;
}