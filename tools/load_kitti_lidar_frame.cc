#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

using std::cout;
using std::endl;
using std::string;
using std::ifstream;
using std::vector;

using std::chrono::milliseconds;
using std::chrono::steady_clock;
using std::chrono::duration_cast;

using pcl::PointXYZI;
using pcl::PointCloud;
using pcl::visualization::CloudViewer;

int main(int argc, char *argv[]){
    string lidar_data_path = argv[1];

    ifstream lidar_data_file(lidar_data_path, ifstream::in | ifstream::binary);

    if(!lidar_data_file){
        return EXIT_FAILURE;
    }

    // Get file size
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    cout << "Begin loading lidar sequence..." << endl;

    steady_clock::time_point t1 = steady_clock::now();

    vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data.front()), 
                         num_elements * sizeof(float));

    steady_clock::time_point t2 = steady_clock::now();
    milliseconds time_used = duration_cast<milliseconds>(t2 - t1);

    cout << "Begin loading lidar sequence... Done" << endl;
    cout << "A total of " << num_elements << " points in this lidar frame" << endl;
    cout << "Time used: " << time_used.count() << " ms." << endl;

    cout << "Converting to pcl point cloud..." << endl;

    t1 = steady_clock::now();

    PointCloud<PointXYZI> lidar_frame;
    for(size_t i = 0; i < lidar_data.size(); i += 4){
        PointXYZI p;
        p.x = lidar_data[i];
        p.y = lidar_data[i + 1];
        p.z = lidar_data[i + 2];
        p.intensity = lidar_data[i + 3];
        lidar_frame.push_back(p);
    }

    t2 = steady_clock::now();
    time_used = duration_cast<milliseconds>(t2 - t1);

    cout << "Converting to pcl point cloud... Done" << endl;
    cout << "Time used: " << time_used.count() << " ms." << endl;

    CloudViewer viewer("LiDAR Frame");
    viewer.showCloud(lidar_frame.makeShared());
    while (!viewer.wasStopped ()){}
    
    return EXIT_SUCCESS;
}
