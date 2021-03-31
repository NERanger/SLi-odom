#include <string>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

#include "sli_slam/Dataset.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Lidar.hpp"
#include "sli_slam/Config.hpp"
#include "sli_slam/Camera.hpp"

using std::string;
using std::cout;
using std::cerr;
using std::endl;

using pcl::PointXYZI;
using pcl::PointCloud;

using cv::Mat;
using cv::imshow;
using cv::waitKey;

using sli_slam::Dataset;
using sli_slam::Frame;
using sli_slam::Lidar;
using sli_slam::Camera;
using sli_slam::Config;

int main(int argc, char const *argv[]){

    if(argc != 2){
        cerr << "Usage: ./RangeImgOnCam <path-to-config>" << endl;
    }

    string config_path(argv[1]);
    if(!Config::SetParameterFile(config_path)){
        cerr << "Cannot open config file at " << config_path << endl;
        return EXIT_FAILURE;
    }

    Dataset dataset(Config::Get<string>("dataset_dir"));

    if(!dataset.Init()){
        cerr << "Dataset loading fails." << endl;
        return EXIT_FAILURE;
    }

    Camera::Ptr left_cam = dataset.GetCameraById(0);
    Lidar::Ptr lidar = dataset.GetLidarById(0);

    while(true){
        Frame::Ptr f = dataset.NextFrame();
        if(!f){
            break;
        }

        Mat left_img = f->LeftImg();
        PointCloud<PointXYZI>::Ptr lidar_xyzi = f->LidarPoints();

        // Convert to left camera coordiante
        lidar->Lidar2LeftCam(lidar_xyzi);

        Mat depth = left_cam->GenDepthMapFromLidar(lidar_xyzi);

        imshow("depth", depth);
        imshow("color", left_img);
        waitKey(0);

    }

    return EXIT_SUCCESS;
}
