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
double ComputeRTE(SE3d gt, SE3d esti, SE3d last_gt, SE3d last_esti);
double ComputeRPE(SE3d gt, SE3d esti, SE3d last_gt, SE3d last_esti);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);
void ToKittiTrajectoryFormat(TrajectoryType &esti);

DEFINE_string(gt, "./dataset/groundtruth.txt", "groundtruth file path");
DEFINE_string(esti, "./dataset/estimated.txt", "estimated file path");

DEFINE_bool(fmt_cvt, false, "If convert to kitti format");

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

    if(FLAGS_fmt_cvt){
        cout << "Convert to KITTI format" << endl;
        ToKittiTrajectoryFormat(estimated);
    }

    // compute rmse&rte
    double rmse = 0.0;
    double rte = 0.0;
    for (size_t i = 0; i < estimated.size(); i++) {
        Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
        double error = (p2.inverse() * p1).log().norm();
        rmse += error * error;

        if(i == 0){
            continue;
        }
        rte += ComputeRTE(groundtruth[i], estimated[i], groundtruth[i-1], estimated[i-1]);
    }

    rmse = rmse / double(estimated.size());
    rmse = sqrt(rmse);
    cout << "Absolute trajectory error (RMSE) = " << rmse << endl;

    rte = rte / double(estimated.size());
    cout << "Average relative translation error = " << rte << " (meter)" << endl;
    DrawTrajectory(groundtruth, estimated);
    
    return EXIT_SUCCESS;
}

void ToKittiTrajectoryFormat(TrajectoryType &esti){
    SE3d Two = esti[0].inverse();
    for(auto &p : esti){
        p = p*Two;
        p = p.inverse();
    }
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


// Compute relative translation error
double ComputeRTE(SE3d gt, SE3d esti, SE3d last_gt, SE3d last_esti){
    SE3d e_gt = last_gt.inverse() * gt;
    SE3d e_esti = last_esti.inverse() * last_esti;

    Vector3d t_gt = e_gt.translation();
    Vector3d t_esti = e_esti.translation();

    double e_x = t_gt[0] - t_esti[0];
    double e_y = t_gt[1] - t_esti[1];
    double e_z = t_gt[2] - t_esti[2];

    return sqrt(e_x*e_x + e_y*e_y + e_z*e_z);
}

// Compute relative pose error
double ComputeRPE(SE3d gt, SE3d esti, SE3d last_gt, SE3d last_esti){
    double e = ((last_gt.inverse() * gt).inverse() * (last_esti.inverse() * esti)).log().norm();
    return e*e;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti){
    // Create pangolin window and plot trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.01, 10000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, pangolin::AxisZ));

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetAspect(640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::DataLog log;

    vector<string> labels;
    labels.push_back(string("Relative Translation Error(meter)"));
    // labels.push_back(string("Relative Pose Error"));
    log.SetLabels(labels);

    pangolin::Plotter plotter(&log, 0, 600, -1, 2.5);
    plotter.SetAspect(640.0f/480.0f);
    plotter.Track("$i");

    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqual)
        .AddDisplay(d_cam)
        .AddDisplay(plotter);

    size_t vis_idx = 1;
    while (pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < vis_idx; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);  // Blue for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < vis_idx; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);  // Red for estimated
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        plotter.Activate(s_cam);

        if(vis_idx < gt.size()){
            log.Log(ComputeRTE(gt[vis_idx], esti[vis_idx], gt[vis_idx - 1], esti[vis_idx - 1]));
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms

        vis_idx += 1;
    }
}