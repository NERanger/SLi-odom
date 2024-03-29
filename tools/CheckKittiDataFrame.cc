#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "sli_slam/Dataset.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Lidar.hpp"
#include "sli_slam/Config.hpp"

using std::string;
using std::cerr;
using std::endl;

using cv::Mat;
using cv::vconcat;
using cv::waitKey;
using cv::imshow;

using pcl::PointXYZ;
using pcl::PointXYZI;
using pcl::PointCloud;
using pcl::visualization::PCLVisualizer;
using pcl::copyPointCloud;
using pcl::visualization::PointCloudColorHandlerCustom;

using sli_slam::Dataset;
using sli_slam::Frame;
using sli_slam::Lidar;
using sli_slam::Config;

// PCL vislulizer ref：
// http://www.pcl-users.org/Simple-animation-with-use-of-pcl-visualization-PCLVisualizer-td4046220.html
// https://pointclouds.org/documentation/tutorials/pcl_visualizer.html
int main(int argc, char *argv[]){
    string dataset_path = argv[1];

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
        cerr << "Dataset init fail." << endl;
        return EXIT_FAILURE;
    }

    Lidar::Ptr lidar = dataset.GetLidarById(0);

    PCLVisualizer::Ptr pts_viewer (new PCLVisualizer ("3D Viewer"));
    pts_viewer->setBackgroundColor(0, 0, 0);
    pts_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    pts_viewer->addCoordinateSystem(1.0);
    pts_viewer->initCameraParameters();

    while (!pts_viewer->wasStopped()){
        Frame::Ptr f = dataset.NextFrame();
        if(!f){
            break;
        }

        Mat left_img, right_img, img;
        PointCloud<PointXYZI>::Ptr lidar_xyzi;

        left_img = f->LeftImg();
        right_img = f->RightImg();
        vconcat(left_img, right_img, img);
        lidar_xyzi = f->LidarPoints();

        lidar->Lidar2LeftCam(lidar_xyzi);

        pts_viewer->removeAllPointClouds();

        pts_viewer->addPointCloud<PointXYZI>(lidar_xyzi);

        pts_viewer->spinOnce(100);

        imshow("Camera", img);
        waitKey(2000);
    }

    return EXIT_SUCCESS;
}