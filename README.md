# Stereo-LiDAR Odometry

A simple LiDAR-assisted stereo odometry

## Prerequisites

We have tested this repo on Ubuntu 18.04 LTS. And it should be easy to compile and run this repo on other platforms.

### Compiler

We require a compiler supporting c++11.

### Pangolin

[Pangolin](https://github.com/stevenlovegrove/Pangolin) is leveraged for visualization.

### OpenCV

[OpenCV 3.4.13](https://github.com/opencv/opencv/archive/3.4.13.zip) is used when we test on our own. You are free to try out other OpenCV versions but the compatibility is not guaranteed.

### Eigen3

Required by g2o, at least 3.1.0 is required.

### g2o

We leverage [g2o](https://github.com/RainerKuemmerle/g2o) to perform graph optimization.

## Build and run

To build:

```shell
mkdir build
cd build
cmake ..
make
```

To run with KITTI dataset sequence 0:

```shell
./bin/run_kitti_stereo -config_file ./config/kitti_00.yaml
```

