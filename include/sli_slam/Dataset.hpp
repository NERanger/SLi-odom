#ifndef SLI_SLAM_DATASET_HPP
#define SLI_SLAM_DATASET_HPP

#include <string>
#include <memory>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sli_slam/Common.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Camera.hpp"
#include "sli_slam/Lidar.hpp"

namespace sli_slam{

/**
 * Dateset loader
 * Pass config file path during construction
 * dataset_dir is the dataset path
 * Get access to camera and next frame after Init()
 */
class Dataset{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef std::shared_ptr<Dataset> Ptr;

    Dataset(const std::string &dataset_path) : dataset_path_(dataset_path){}

    /**
     * Initializaiton
     * @return if success
     */
    bool Init();

    // Create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    // Get camera by id
    Camera::Ptr GetCameraById(int camera_id) const {return cameras_.at(camera_id);}

    // Get Lidar by id
    Lidar::Ptr GetLidarById(int lidar_id) const {return lidars_.at(lidar_id);}

private:
    std::string dataset_path_;
    int current_frame_index_ = 0;

    // Image scale factor
    double img_scale_ = 0.5;

    // Two cameras when using KITTI
    // cam0 is left cam
    // cam1 is right cam
    std::vector<Camera::Ptr> cameras_;
    std::vector<Lidar::Ptr> lidars_;  // One LiDAR when using KITTI

    pcl::PointCloud<pcl::PointXYZI>::Ptr LoadKittiLidarFrame(const std::string &path);
};

}

#endif