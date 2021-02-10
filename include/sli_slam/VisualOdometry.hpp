#ifndef SLI_SLAM_VISUAL_ODOMETRY_HPP
#define SLI_SLAM_VISUAL_ODOMETRY_HPP

#include <memory>
#include <string>

#include "sli_slam/Backend.hpp"
#include "sli_slam/Common.hpp"
#include "sli_slam/Dataset.hpp"
#include "sli_slam/Frontend.hpp"
#include "sli_slam/Viewer.hpp"

namespace sli_slam{

class VisualOdometry{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    VisualOdometry(std::string &config_path) : config_file_path_(config_path){};

    /**
     * Initialization before running
     * @return true if success
     */
    bool Init();

    /**
     * Start visual odometry with given dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    FrontendStatus GetFrontendStatus() const {return frontend_->GetStatus();}

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // Dataset
    Dataset::Ptr dataset_ = nullptr;
};

}

#endif