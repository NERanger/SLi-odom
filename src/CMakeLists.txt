add_library(
    sli_slam SHARED
        sli_slam/Algorithm.cc
        sli_slam/Backend.cc
        sli_slam/Camera.cc
        sli_slam/Config.cc
        sli_slam/Dataset.cc
        sli_slam/Frame.cc
        sli_slam/Frontend.cc
        sli_slam/G2oTypes.cc
        sli_slam/Lidar.cc
        sli_slam/Map.cc
        sli_slam/MapPoint.cc
        sli_slam/Viewer.cc
        sli_slam/VisualOdometry.cc)

target_link_libraries(sli_slam ${THIRD_PARTY_LIBS})