#include <fstream>

#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include <glog/logging.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "sli_slam/Dataset.hpp"
#include "sli_slam/Frame.hpp"

using std::ifstream;

using boost::format;

using Sophus::SE3d;
using Sophus::SO3d;

using cv::Mat;
using cv::Size;
using cv::imread;
using cv::resize;

using sli_slam::Dataset;
using sli_slam::Frame;

bool Dataset::Init(){
    // Read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");
    if(!fin){
        LOG(ERROR) << "Cannot find " << dataset_path_ << "/calib.txt";
        return false;
    }

    for(int i = 0; i < 4; ++i){
        char camera_name[3];
        for(int k = 0; k < 3; ++k){
            fin >> camera_name[k];
        }

        double projection_data[12];
        for(int k = 0; k < 12; ++k){
            fin >> projection_data[k];
        }

        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
             projection_data[4], projection_data[5], projection_data[6],
             projection_data[8], projection_data[9], projection_data[10];

        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;

        // New intrinsics after downsampling with factor 0.5
        K = K * 0.5;

        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3d(SO3d(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }

    fin.close();
    current_image_index_ = 0;

    return true;
}

Frame::Ptr Dataset::NextFrame(){
    format fmt("%s/image_%d/%06d.png");
    Mat image_left, image_right;

    // Load images
    image_left = imread((fmt % dataset_path_ % 0 % current_image_index_).str(), 
                        cv::IMREAD_GRAYSCALE);
    image_right = imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                         cv::IMREAD_GRAYSCALE);

    if(image_left.data == nullptr || image_right.data == nullptr){
        LOG(WARNING) << "cannot find image at index " << current_image_index_;
        return nullptr;
    }

    Mat image_left_resized, image_right_resized;
    resize(image_left, image_left_resized, Size(), 0.5, 0.5, cv::INTER_NEAREST);
    resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    Frame::Ptr new_frame = Frame::CreateFrame();
    new_frame->SetLeftImg(image_left_resized);
    new_frame->SetRightImg(image_right_resized);
    current_image_index_ += 1;

    return new_frame;
}