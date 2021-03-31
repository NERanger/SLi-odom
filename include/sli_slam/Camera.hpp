#ifndef SLI_SLAM_CAMERA_HPP
#define SLI_SLAM_CAMERA_HPP

#include <memory>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"

namespace sli_slam{

// Default as pinhole stereo camera model
class Camera{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    Camera() = default;
    Camera(double fx, double fy, double cx, double cy, int img_width, int img_height, 
           double baseline, Mat34 &projection_mat, const Sophus::SE3d &pose);

    Sophus::SE3d Pose() const {return pose_;}
    Mat34 ProjectionMat() const {return projection_mat_;}
    Mat33 GetIntrinsicMatrix() const;

    cv::Mat GenDepthMapFromLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr pt_cloud);

    // Coordinate transform: world, camera, pixel
    Vec3 World2Camera(const Vec3 &p_w, const Sophus::SE3d &T_c_w);
    Vec3 Camera2World(const Vec3 &p_c, const Sophus::SE3d &T_c_w);
    Vec2 Camera2Pixel(const Vec3 &p_c);
    Vec3 Pixel2Camera(const Vec2 &p_p, double depth = 1);
    Vec3 Pixel2World(const Vec2 &p_p, const Sophus::SE3d &T_c_w, double depth = 1.0);
    Vec2 World2Pixel(const Vec3 &p_w, const Sophus::SE3d &T_c_w);

private:

    // Camera intrinsics
    // Assume same intrinsics on left and right camera
    double fx_ = 0;
    double fy_ = 0;
    double cx_ = 0;
    double cy_ = 0;

    // Stereo baseline length
    double baseline_ = 0;

    int img_width_ = 0;
    int img_height_ = 0;

    // Rectified projection matrix
    Mat34 projection_mat_;

    Sophus::SE3d pose_;      // Extrinsics
    Sophus::SE3d pose_inv_;  // Extrinsics inverse
};

} // namespace sli_slam

#endif