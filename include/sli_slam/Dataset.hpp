#ifndef SLI_SLAM_DATASET_HPP
#define SLI_SLAM_DATASET_HPP

#include <string>
#include <memory>
#include <vector>

#include "sli_slam/Common.hpp"
#include "sli_slam/Frame.hpp"
#include "sli_slam/Camera.hpp"

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

private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};

}

#endif