#include <fstream>
#include <vector>

#include <boost/format.hpp>

#include <opencv2/opencv.hpp>

#include <glog/logging.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

#include "sli_slam/Dataset.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Lidar.hpp"
#include "sli_slam/Config.hpp"

using std::ifstream;
using std::vector;

using boost::format;

using Eigen::Quaterniond;

using Sophus::SE3d;
using Sophus::SO3d;

using cv::Mat;
using cv::Size;
using cv::imread;
using cv::resize;

using pcl::PointCloud;
using pcl::PointXYZI;
using pcl::removeNaNFromPointCloud;

using sli_slam::Dataset;
using sli_slam::Frame;
using sli_slam::Lidar;
using sli_slam::Config;

namespace{
    const double kMinimumLidarPointRange = 0.1;
}

bool Dataset::Init(){
    // Read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");
    if(!fin){
        LOG(ERROR) << "Cannot find " << dataset_path_ << "/calib.txt";
        return false;
    }

    int img_width = Config::Get<int>("img_width") * img_scale_;
    int img_height = Config::Get<int>("img_height") * img_scale_;

    // Load camera projection matrix
    for(int i = 0; i < 4; ++i){
        char camera_name[3];
        for(int k = 0; k < 3; ++k){
            fin >> camera_name[k];
        }

        double projection_data[12];
        for(int k = 0; k < 12; ++k){
            fin >> projection_data[k];
        }

        Mat34 P;
        P << projection_data[0], projection_data[1], projection_data[2], projection_data[3],
             projection_data[4], projection_data[5], projection_data[6], projection_data[7],
             projection_data[8], projection_data[9], projection_data[10], projection_data[11];

        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
             projection_data[4], projection_data[5], projection_data[6],
             projection_data[8], projection_data[9], projection_data[10];

        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;

        // New intrinsics after downsampling with factor 0.5
        K = K * img_scale_;

        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), img_width, img_height,
                                          t.norm(), P, SE3d(SO3d(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }

    // Load LiDAR extrinsics (T_c_li)
    char trans_prefix[3];
    for(int k = 0; k < 3; ++k){
        fin >> trans_prefix[k];
    }

    double lidar_trans_data[12];
    for(int k = 0; k < 12; ++k){
        fin >> lidar_trans_data[k];
    }

    Mat33 R_c_li;
    R_c_li << lidar_trans_data[0], lidar_trans_data[1], lidar_trans_data[2],
              lidar_trans_data[4], lidar_trans_data[5], lidar_trans_data[6],
              lidar_trans_data[8], lidar_trans_data[9], lidar_trans_data[10];

    Vec3 t_c_li;
    t_c_li << lidar_trans_data[3], lidar_trans_data[7], lidar_trans_data[11];

    Quaterniond Q_c_li(R_c_li);
    SE3d T_c_li(Q_c_li, t_c_li);
    Lidar::Ptr new_lidar(new Lidar(T_c_li));
    lidars_.push_back(new_lidar);

    LOG(INFO) << "T_c_li: " << T_c_li.matrix3x4();

    fin.close();
    current_frame_index_ = 0;

    return true;
}

Frame::Ptr Dataset::NextFrame(){
    format img_fmt("%s/image_%d/%06d.png");
    format lidar_fmt("%s/velodyne/%06d.bin");

    // Load images
    Mat image_left, image_right;
    image_left = imread((img_fmt % dataset_path_ % 0 % current_frame_index_).str(), 
                        cv::IMREAD_GRAYSCALE);
    image_right = imread((img_fmt % dataset_path_ % 1 % current_frame_index_).str(),
                         cv::IMREAD_GRAYSCALE);

    if(image_left.data == nullptr || image_right.data == nullptr){
        LOG(WARNING) << "cannot find image at index " << current_frame_index_;
        return nullptr;
    }

    Mat image_left_resized, image_right_resized;
    resize(image_left, image_left_resized, Size(), 0.5, 0.5, cv::INTER_NEAREST);
    resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    // Load LiDAR pointcloud
    PointCloud<PointXYZI>::Ptr lidar_frame = 
        LoadKittiLidarFrame((lidar_fmt % dataset_path_ % current_frame_index_).str());

    std::vector<int> indices; // For function usage, no actual need
    removeNaNFromPointCloud(*lidar_frame, *lidar_frame, indices);
    Lidar::RemoveClosePoint(lidar_frame, kMinimumLidarPointRange);

    if(lidar_frame->empty()){
        LOG(WARNING) << "Empty LiDAR frame at index " << current_frame_index_;
        return nullptr;
    }

    Frame::Ptr new_frame = Frame::CreateFrame();
    new_frame->SetLeftImg(image_left_resized);
    new_frame->SetRightImg(image_right_resized);
    new_frame->SetLidarPoints(lidar_frame);
    current_frame_index_ += 1;

    return new_frame;
}

PointCloud<PointXYZI>::Ptr Dataset::LoadKittiLidarFrame(const std::string &path){
    ifstream lidar_data_file(path, ifstream::in | ifstream::binary);

    // Get file size
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data.front()), 
                         num_elements * sizeof(float));

    PointCloud<PointXYZI> lidar_frame;
    PointCloud<PointXYZI>::Ptr lidar_frame_ptr = lidar_frame.makeShared();
    for(size_t i = 0; i < lidar_data.size(); i += 4){
        PointXYZI p;
        p.x = lidar_data[i];
        p.y = lidar_data[i + 1];
        p.z = lidar_data[i + 2];
        p.intensity = lidar_data[i + 3];
        lidar_frame_ptr->push_back(p);
    }

    return lidar_frame_ptr;
}