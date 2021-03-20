#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <sli_slam/Dataset.hpp>
#include <sli_slam/Frame.hpp>

using std::string;
using std::cerr;
using std::endl;

using cv::Mat;
using cv::vconcat;
using cv::waitKey;
using cv::imshow;

using pcl::PointXYZI;
using pcl::PointCloud;
using pcl::visualization::CloudViewer;

using sli_slam::Dataset;
using sli_slam::Frame;

int main(int argc, char *argv[]){
    string dataset_path = argv[1];

    Dataset dataset(dataset_path);

    if(!dataset.Init()){
        cerr << "Dataset init fail." << endl;
        return EXIT_FAILURE;
    }

    while (true){
        Frame::Ptr f = dataset.NextFrame();
        if(!f){
            break;
        }

        Mat left_img, right_img, img;
        PointCloud<PointXYZI>::Ptr lidar;

        left_img = f->LeftImg();
        right_img = f->RightImg();
        vconcat(left_img, right_img, img);
        lidar = f->LidarPoints();

        CloudViewer viewer("LiDAR Frame");
        viewer.showCloud(lidar);

        imshow("Camera", img);
        waitKey(20);
    }

    return EXIT_SUCCESS;
}
