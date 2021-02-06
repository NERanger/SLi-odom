#include <vector>
#include <sophus/se3.hpp>

#include "sli_slam/Common.hpp"
#include "sli_slam/Algorithm.hpp"

using std::vector;

using Eigen::BDCSVD;
using Eigen::MatrixXd;

using Sophus::SE3d;

// Reference (Not exactly same problem): 
// http://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
// https://www.ecse.rpi.edu/~qji/CV/svd_review.pdf
// https://www.cse.unr.edu/~bebis/MathMethods/SVD/lecture.pdf
// For further improvement, depth filter can be considered
bool sli_slam::Triangulation(const vector<SE3d> &poses, 
                             const vector<Vec3> points, 
                             Vec3 &pt_world) {

    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }

    BDCSVD<MatrixXd> svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // Bad solution, discard
        return false;
    }
    
    return true;
}