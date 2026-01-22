# Guide: Intel RealSense D435i Integration on Qualcomm RB5 with ROS2 Foxy

**M2 Intelligent Systems Engineering - Sorbonne**

---

## Table of Contents

1. [Introduction](#introduction)
2. [System Configuration](#system-configuration)
3. [Problem Statement](#problem-statement)
4. [System Preparation](#system-preparation)
5. [Librealsense2 Compilation](#librealsense2-compilation)
6. [ROS2 Wrapper Installation](#ros2-wrapper-installation)
7. [Device Identification](#device-identification)
8. [Final Solution: Direct V4L2 Access](#final-solution-direct-v4l2-access)
9. [ROS2 Integration](#ros2-integration)

---

## Introduction

This guide provides step-by-step instructions for integrating the Intel RealSense D435i depth camera with the Qualcomm Robotics RB5 Development Kit running Ubuntu 20.04 and ROS2 Foxy. Due to kernel limitations on the RB5, standard librealsense2 integration does not work, requiring alternative approaches.

## System Configuration

- Qualcomm RB5 Development Kit
- Intel RealSense D435i depth camera
- USB C connection


## Problem Statement

The RB5 kernel lacks RealSense-specific patches, causing V4L2 control failures:

- Error: `xioctl(VIDIOC_S_EXT_CTRLS) failed`
- Error: `DS5 group_devices is empty`
- Pre-compiled Intel packages unavailable for ARM64

## System Preparation

### Install Build Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    git libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev \
    libglu1-mesa-dev cmake build-essential \
    libudev-dev v4l-utils
```

### Install ROS2 Dependencies

```bash
sudo apt-get install -y \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-diagnostic-updater
```

## Librealsense2 Compilation

### Clone Repository

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

### Build Configuration

Create build directory and configure:

```bash
mkdir build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### Compile and Install

```bash
make -j4
sudo make install
sudo ldconfig
```

**Note:** Compilation takes approximately 15-30 minutes on RB5.

### Configure udev Rules

```bash
cd ~/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Update Environment

```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
```

### Fix Python Bindings

Create symbolic link for pyrealsense2:

```bash
sudo ln -s /usr/local/OFF/pyrealsense2.cpython-38-aarch64-linux-gnu.so \
    /usr/local/lib/python3.8/dist-packages/pyrealsense2.so
```

## ROS2 Wrapper Installation

### Clone realsense-ros Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd realsense-ros
git checkout 4c9b0db
```

### Fix Version Compatibility

Modify CMakeLists.txt to match installed librealsense2 version:

```bash
cd ~/ros2_ws/src/realsense-ros/realsense2_camera
sed -i 's/find_package(realsense2 2.56.4 REQUIRED)/find_package(realsense2 2.54.2 REQUIRED)/' CMakeLists.txt
```

### Build ROS2 Packages

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select realsense2_camera_msgs realsense2_camera realsense2_description
source install/setup.bash
```

## Device Identification

```bash
rs-enumerate-devices 
```
