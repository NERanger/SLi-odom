#include <vector>

#include <gtest/gtest.h>

#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"
#include "sli_slam/Algorithm.hpp"

using std::vector;

using Sophus::SE3d;

using sli_slam::Triangulation;

TEST(sli_slam_test, Triangulation){
    Vec3 pt_world(30, 20, 10);
    Vec3 pt_world_estimated;
    
    vector<SE3d> poses{
        SE3d(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 0, 0)),
        SE3d(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, -10, 0)),
        SE3d(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 10, 0)),
    };

    vector<Vec3> points;
    for(size_t i = 0; i < poses.size(); ++i){
        Vec3 pc = poses[i] * pt_world;
        pc /= pc[2];
        points.push_back(pc);
    }

    EXPECT_TRUE(Triangulation(poses, points, pt_world_estimated));

    // std::cout << pt_world_estimated << std::endl;

    EXPECT_NEAR(pt_world[0], pt_world_estimated[0], 0.01);
    EXPECT_NEAR(pt_world[1], pt_world_estimated[1], 0.01);
    EXPECT_NEAR(pt_world[2], pt_world_estimated[2], 0.01);
}

int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
