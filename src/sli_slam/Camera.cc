#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"
#include "sli_slam/Camera.hpp"

using Sophus::SE3d;

using sli_slam::Camera;

Camera::Camera(double fx, double fy, double cx, double cy, 
               double baseline, const SE3d &pose) : 
               fx_(fx), fy_(fy), cx_(cx), cy_(cy),
               baseline_(baseline), pose_(pose){
    
    pose_inv_ = pose_.inverse();
}

Mat33 Camera::GetIntrinsicMatrix() const{
    Mat33 k;
    k << fx_, 0  , cx_, 
         0  , fy_, cy_,
         0  , 0  , 1;

    return k;
}

Vec3 Camera::World2Camera(const Vec3 &p_w, const SE3d &T_c_w){
    return pose_ * T_c_w * p_w;
}

Vec3 Camera::Camera2World(const Vec3 &p_c, const SE3d &T_c_w){
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Vec2 Camera::Camera2Pixel(const Vec3 &p_c){
    return Vec2( fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
                 fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vec3 Camera::Pixel2Camera(const Vec2 &p_p, double depth){
    return Vec3((p_p(0, 0) - cx_) * depth / fx_,
                (p_p(1, 0) - cy_) * depth / fy_,
                depth);
}

Vec3 Camera::Pixel2World(const Vec2 &p_p, const SE3d &T_c_w, double depth){
    return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
}

Vec2 Camera::World2Pixel(const Vec3 &p_w, const SE3d &T_c_w){
    return Camera2Pixel(World2Camera(p_w, T_c_w));
}