#include "sli_slam/G2oTypes.hpp"

using sli_slam::VertexPose;
using sli_slam::VertexXYZ;
using sli_slam::EdgeProjection;
using sli_slam::EdgeProjectionPoseOnly;

void VertexPose::setToOriginImpl() { 
    _estimate = Sophus::SE3d(); 
}

void VertexPose::oplusImpl(const double *update){
    Vec6 update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
}

bool VertexPose::read(std::istream &in){ 
    return true; 
}
bool VertexPose::write(std::ostream &out) const {
    return true; 
}

void VertexXYZ::setToOriginImpl(){
    _estimate = Vec3::Zero();
}

void VertexXYZ::oplusImpl(const double *update){
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
}

bool VertexXYZ::read(std::istream &in){ 
    return true; 
}
bool VertexXYZ::write(std::ostream &out) const {
    return true; 
}

void EdgeProjectionPoseOnly::computeError(){
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Vec3 pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
}

void EdgeProjectionPoseOnly::linearizeOplus(){
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Vec3 pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << 
        -fx * Zinv              , 0                      , fx * X * Zinv2     , fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv          , 0                  , -fy * Zinv,
        fy * Y * Zinv2          , fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;
}

bool EdgeProjectionPoseOnly::read(std::istream &in){ 
    return true; 
}
bool EdgeProjectionPoseOnly::write(std::ostream &out) const {
    return true; 
}

void EdgeProjection::computeError(){
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
}

void EdgeProjection::linearizeOplus(){
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Vec3 pw = v1->estimate();
    Vec3 pos_cam = _cam_ext * T * pw;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << 
        -fx * Zinv              , 0                      , fx * X * Zinv2     , fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv          , 0                  , -fy * Zinv,
        fy * Y * Zinv2          , fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;

    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                        _cam_ext.rotationMatrix() * T.rotationMatrix();
}

bool EdgeProjection::read(std::istream &in){ 
    return true; 
}
bool EdgeProjection::write(std::ostream &out) const {
    return true; 
}