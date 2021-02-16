add_executable(trajectory_visualizer TrajectoryVisualizer.cc)
target_link_libraries(
    trajectory_visualizer 
        ${GFLAGS_LIBRARIES} 
        Sophus::Sophus
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        pthread)