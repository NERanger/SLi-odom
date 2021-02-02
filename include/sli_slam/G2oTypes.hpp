#ifndef SLI_SLAM_G2O_TYPE_HPP
#define SLI_SLAM_G2O_TYPE_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"

namespace sli_slam{

// Vertex for pose
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override;
    virtual void oplusImpl(const double *update) override;

    virtual bool read(std::istream &in) override {return true;}
    virtual bool write(std::ostream &out) const override {return true;}

};

// Vertex for landmark
class VertexXYZ : public g2o::BaseVertex<3, Vec3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override;
    virtual void oplusImpl(const double *update) override;

    virtual bool read(std::istream &in) override {return true;}
    virtual bool write(std::ostream &out) const override {return true;}
};

// Unary edge only related with pose
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // pos: landmark position
    EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K) : _pos3d(pos), _K(K) {}

    virtual void computeError() override;
    virtual void linearizeOplus() override;

    virtual bool read(std::istream &in) override {return true;}
    virtual bool write(std::ostream &out) const override {return true;}

private:
    Vec3 _pos3d;
    Mat33 _K;
};

// Binary edge with landmark and pose
class EdgeProjection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // Pass extrinsics and intrinsics during construction
    EdgeProjection(const Mat33 &K, const Sophus::SE3d &cam_ext) : _K(K), _cam_ext(cam_ext) {}

    virtual void computeError() override;
    virtual void linearizeOplus() override;

    virtual bool read(std::istream &in) override {return true;}
    virtual bool write(std::ostream &out) const override {return true;}

private:
    Mat33 _K;
    Sophus::SE3d _cam_ext;
};

}

#endif