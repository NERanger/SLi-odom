#include <pcl/common/transforms.h>

#include "sli_slam/Lidar.hpp"

using pcl::PointCloud;
using pcl::PointXYZI;
using pcl::transformPointCloud;

using sli_slam::Lidar;

PointCloud<PointXYZI>::Ptr Lidar::Lidar2LeftCam(PointCloud<PointXYZI>::Ptr &cloud) const{
    PointCloud<PointXYZI>::Ptr transformed_ptcloud(new PointCloud<PointXYZI>);

    transformPointCloud(*cloud, *transformed_ptcloud, pose_.matrix());

    // cloud = transformed_ptcloud;
    return transformed_ptcloud;
}

void Lidar::RemoveCloseFarPoint(PointCloud<PointXYZI>::Ptr &cloud, double close_thresh, double far_thresh){
    PointCloud<PointXYZI>::Ptr out_ptr(new PointCloud<PointXYZI>);

    uint32_t width = 0;
    for(size_t i = 0; i < cloud->points.size(); ++i){
        double dis_squared = cloud->points[i].x * cloud->points[i].x +
                             cloud->points[i].y * cloud->points[i].y +
                             cloud->points[i].z * cloud->points[i].z;
        if(dis_squared < close_thresh * close_thresh || dis_squared > far_thresh * far_thresh){
            // std::cout << "Point removed " << dis_squared << std::endl;
            continue;
        }
        out_ptr->points.push_back(cloud->points[i]);
        width += 1;
    }

    out_ptr->height = 1;
    out_ptr->width = width;
    out_ptr->is_dense = true;

    cloud = out_ptr;
}