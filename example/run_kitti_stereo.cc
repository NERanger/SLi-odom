#include <gflags/gflags.h>
#include <glog/logging.h>

#include "sli_slam/VisualOdometry.hpp"

DEFINE_string(config_file, "./config/kitti_00.yaml", "config file path");

int main(int argc, char **argv){
    google::ParseCommandLineFlags(&argc, &argv, true);
    // google::InitGoogleLogging(argv[0]);

    sli_slam::VisualOdometry::Ptr visual_odom(
        new sli_slam::VisualOdometry(FLAGS_config_file));

    assert(visual_odom->Init() == true);
    visual_odom->Run();

    return EXIT_SUCCESS;
}
