#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <unistd.h>

#include <pangolin/pangolin.h>

#include <gflags/gflags.h>

#include <sophus/se3.hpp>

using std::vector;
using std::string;
using std::ifstream;
using std::cout;
using std::cerr;
using std::endl;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

using Sophus::SE3d;

typedef vector<SE3d, Eigen::aligned_allocator<SE3d>> TrajectoryType;

TrajectoryType ReadTrajectory(const string &path);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

DEFINE_string(gt, "./config/default.yaml", "groundtruth file path");
DEFINE_string(esti, "./config/default.yaml", "estimated file path");

int main(int argc, char **argv){
    google::ParseCommandLineFlags(&argc, &argv, true);

    cout << "Loading groundtruth..." << endl;
    TrajectoryType groundtruth = ReadTrajectory(FLAGS_gt);
    cout << "Loading groundtruth... Done" << endl;

    cout << "Loading estimated..." << endl;
    TrajectoryType estimated = ReadTrajectory(FLAGS_esti);
    cout << "Loading estimated... Done" << endl;
    
    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    // compute rmse
    double rmse = 0;
    for (size_t i = 0; i < estimated.size(); i++) {
        Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
        double error = (p2.inverse() * p1).log().norm();
        rmse += error * error;
    }
    rmse = rmse / double(estimated.size());
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    DrawTrajectory(groundtruth, estimated);
    
    return EXIT_SUCCESS;
}


TrajectoryType ReadTrajectory(const string &path) {
    ifstream fin(path);
    TrajectoryType trajectory;

    if(!fin){
        cerr << "Trajectory " << path << " not found." << endl;
        return trajectory;
    }

    while(!fin.eof()){
        double t00, t01, t02, t03,
               t10, t11, t12, t13,
               t20, t21, t22, t23;

        fin >> t00 >> t01 >> t02 >> t03
            >> t10 >> t11 >> t12 >> t13
            >> t20 >> t21 >> t22 >> t23;

        Matrix3d R;
        R << t00, t01, t02,
             t10, t11, t12,
             t20, t21, t22;

        Quaterniond q(R);

        Vector3d t;
        t << t03, t13, t23;

        SE3d pose(q, t);
        trajectory.push_back(pose);
    }
    return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti){
    // Create pangolin window and plot trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);  // Blue for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);  // Red for estimated
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}