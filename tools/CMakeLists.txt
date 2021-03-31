add_executable(trajectory_visualizer TrajectoryVisualizer.cc)
target_link_libraries(
    trajectory_visualizer 
        ${GFLAGS_LIBRARIES} 
        Sophus::Sophus
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        pthread)

add_executable(load_kitti_lidar_frame LoadKittiLidarFrame.cc)
target_link_libraries (load_kitti_lidar_frame ${PCL_LIBRARIES})

add_executable(check_kitti_data_frame CheckKittiDataFrame.cc)
target_link_libraries (check_kitti_data_frame sli_slam ${THIRD_PARTY_LIBS})

add_executable(range_img_on_cam RangeImgOnCam.cc)
target_link_libraries (range_img_on_cam sli_slam ${THIRD_PARTY_LIBS})