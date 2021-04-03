#ifndef SLI_SLAM_LIDAR_HPP
#define SLI_SLAM_LIDAR_HPP

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"

namespace sli_slam{

class Lidar{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Lidar> Ptr;

    Lidar() = default;
    Lidar(const Sophus::SE3d &pose){pose_ = pose;}

    // Transform LiDAR pointcloud to left camera coordinate
    // void Lidar2LeftCam(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Lidar2LeftCam(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;

    // Transform LiDAR pointcloud to world coordinate
    void Lidar2World(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;

    static void RemoveCloseFarPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double close_thresh, double far_thresh);
private:
    // T_c_li
    // Tranformation form LiDAR to left camera
    Sophus::SE3d pose_;
};

}

#endif