add_executable(trajectory_visualizer TrajectoryVisualizer.cc)
target_link_libraries(
    trajectory_visualizer 
        ${GFLAGS_LIBRARIES} 
        Sophus::Sophus
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        pthread)

add_executable(load_kitti_lidar_frame load_kitti_lidar_frame.cc)
target_link_libraries (load_kitti_lidar_frame ${PCL_LIBRARIES})