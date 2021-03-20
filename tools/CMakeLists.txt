add_executable(trajectory_visualizer TrajectoryVisualizer.cc)
target_link_libraries(
    trajectory_visualizer 
        ${GFLAGS_LIBRARIES} 
        Sophus::Sophus
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        pthread)

add_executable(load_kitti_lidar_frame load_kitti_lidar_frame.cc)
target_link_libraries (load_kitti_lidar_frame ${PCL_LIBRARIES})

add_executable(check_kitti_data_frame check_kitti_data_frame.cc)
target_link_libraries (check_kitti_data_frame sli_slam ${THIRD_PARTY_LIBS})