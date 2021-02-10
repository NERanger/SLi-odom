#include <thread>
#include <memory>

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include <sophus/se3.hpp>

#include <pangolin/pangolin.h>

#include "sli_slam/Viewer.hpp"

using std::thread;
using std::bind;
using std::mutex;
using std::lock_guard;

using cv::Mat;
using cv::imshow;
using cv::waitKey;
using cv::cvtColor;
using cv::circle;
using cv::Scalar;

using pangolin::OpenGlRenderState;
using pangolin::CreateWindowAndBind;
using pangolin::ProjectionMatrix;
using pangolin::ModelViewLookAt;
using pangolin::View;
using pangolin::CreateDisplay;
using pangolin::Handler3D;
using pangolin::ShouldQuit;
using pangolin::OpenGlMatrix;
using pangolin::FinishFrame;

using Sophus::SE3d;

using sli_slam::Viewer;
using sli_slam::Frame;

Viewer::Viewer(){
    viewer_thread_ = thread(bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close(){
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame){
    lock_guard<mutex> lock(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap(){
    lock_guard<mutex> lock(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetAllActiveKeyFrames();
    active_landmarks_ = map_->GetAllActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop(){
    CreateWindowAndBind("SLi-SLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    OpenGlRenderState vis_camera(
        ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D handler
    View& vis_dispaly = CreateDisplay()
                        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                        .SetHandler(new Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while(!ShouldQuit() && viewer_running_){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_dispaly.Activate(vis_camera);

        lock_guard<mutex> lock(viewer_data_mutex_);
        if(current_frame_){
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            Mat img = PlotFeatureOnImage();
            imshow("Left cam img", img);
            waitKey(1);
        }

        if(map_){
            DrawMapPoints();
        }

        FinishFrame();
        usleep(5000);
    }
    LOG(INFO) << "Viewer stopped";
}

Mat Viewer::PlotFeatureOnImage(){
    Mat img_out;
    cvtColor(current_frame_->LeftImg(), img_out, CV_GRAY2BGR);
    for(size_t i = 0; i < current_frame_->FeatureLeft().size(); ++i){
        if(current_frame_->FeatureLeft()[i]->RelatedMapPoint().lock()){
            Feature::Ptr feat = current_frame_->FeatureLeft()[i];
            circle(img_out, feat->Position().pt, 2, Scalar(0, 250, 0), 2);
        }
    }
    return img_out;
}

void Viewer::FollowCurrentFrame(OpenGlRenderState &vis_camera){
    SE3d Twc = current_frame_->Pose().inverse();
    OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(Frame::Ptr frame, const float* color){
    SE3d Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    // See this website for explanation for 'template'
    // https://stackoverflow.com/questions/8463368/template-dot-template-construction-usage
    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if(color == nullptr){
        glColor3f(1, 0, 0);
    }else{
        glColor3f(color[0], color[1], color[2]);
    }

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints(){
    const float red[3] = {1.0, 0.0, 0.0};
    for(auto &kf : active_keyframes_){
        DrawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for(auto &landmark : active_landmarks_){
        auto pos = landmark.second->Position();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }

    glEnd();
}