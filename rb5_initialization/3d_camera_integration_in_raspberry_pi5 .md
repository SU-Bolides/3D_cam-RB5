# Intel RealSense D435i Installation Guide

**Configuration on Qualcomm raspberry pi5 with ROS2 Jazzy - Ubuntu 24.04**

**Author:** Melissa

---

## Table of Contents

1. [Introduction](#introduction)
2. [librealsense Installation](#librealsense-installation)
3. [Camera Testing](#camera-testing)
4. [ROS2 Wrapper Installation](#ros2-wrapper-installation)
5. [ROS2 Launch and Testing](#ros2-launch-and-testing)
6. [Advanced Configuration](#advanced-configuration)
7. [Troubleshooting](#troubleshooting)
8. [Integration into a Launch File](#integration-into-a-launch-file)
9. [Conclusion](#conclusion)

---

## Introduction

This document details the complete installation of the Intel RealSense D435i camera on a raspberry pi5 running Ubuntu 24.04 and ROS2 Jazzy.

- **Platform:** raspberry pi 5
- **OS:** Ubuntu 24.04 LTS
- **ROS2:** Jazzy 
- **Camera:** Intel RealSense D435i

### Prerequisites

- USB cable to connect the D435i
- Sufficient disk space (~2 GB)

---

## librealsense Installation

### Repository Cloning

```bash
cd ~/bolide_voiture/src/camera
git clone https://github.com/IntelRealSense/librealsense.git
```

### Configuring udev Rules

The udev rules allow camera access without root privileges:

```bash
cd ~/bolide_voiture/src/camera/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Compilation and Installation

#### CMake Configuration

```bash
cd ~/bolide_voiture/src/camera/librealsense
sudo rm -rf build  # Clean if necessary
mkdir build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false
```

> **Warning:** The `-DBUILD_EXAMPLES=false` option is essential because the examples require OpenGL which is not properly configured. Without this option, compilation fails.

#### Compilation

```bash
make -j4
```
#### Installation

```bash
sudo make install
```

Files are installed in:

- Libraries: `/usr/local/lib/`
- Headers: `/usr/local/include/librealsense2/`
- Tools: `/usr/local/bin/rs-*`

### Environment Configuration

Add the library path to `LD_LIBRARY_PATH`:

```bash
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' \
    >> ~/.bashrc
source ~/.bashrc
```

---

## Camera Testing

### Connection and Detection

Connect the D435i camera via USB and verify detection:

```bash
rs-enumerate-devices
```

#### Expected Output

```
Device info: 
    Name                          : Intel RealSense D435I
    Serial Number                 : 047422070094
    Firmware Version              : 5.17.0.10
    Recommended Firmware Version  : 5.17.0.10
    Physical Port                 : /sys/devices/platform/...
    Advanced Mode                 : YES
    Product Id                    : 0B3A
    Connection Type               : USB

Stream Profiles supported by Stereo Module
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Infrared 1  1280x720       Y8          @ 6 Hz
    Infrared 1   848x480       Y8          @ 10/8/6 Hz
    Infrared 1   640x480       Y8          @ 30/15/6 Hz
    ...
    Depth       1280x720       Z16         @ 6 Hz
    Depth        640x480       Z16         @ 30/15/6 Hz
    ...

Stream Profiles supported by RGB Camera
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Color       1920x1080      RGB8        @ 8 Hz
    Color       1280x720       RGB8        @ 15/10/6 Hz
    Color        640x480       RGB8        @ 30/15/6 Hz
    ...

Stream Profiles supported by Motion Module
 Supported modes:
    STREAM      FORMAT         FPS
    Accel       MOTION_XYZ32F  @ 250/63 Hz
    Gyro        MOTION_XYZ32F  @ 400/200 Hz
```

> **Success:** If you see this output, the camera is correctly installed and functional!

---

## ROS2 Wrapper Installation

### ROS2 Repository Cloning

```bash
cd ~/bolide_voiture/src
git clone https://github.com/IntelRealSense/realsense-ros.git \
    -b ros2-development
```

### Installing diagnostic_updater

The `diagnostic_updater` package is required for ROS2:

```bash
sudo apt update
sudo apt install -y ros-jazzy-diagnostic-updater
```

> **Warning:** Verify that you install the `ros-jazzy` version and not `ros-foxy`. The ROS2 distribution must match your installation.

### Compilation with colcon

```bash
cd ~/bolide_voiture
colcon build --packages-select \
    realsense2_camera \
    realsense2_camera_msgs
```

Compilation time: approximately 2 minutes.

---

## ROS2 Launch and Testing

### Launching the Camera Node

```bash
source ~/bolide_voiture/install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### Topic Verification

In a second SSH terminal:

```bash
source ~/bolide_voiture/install/setup.bash

# List all topics
ros2 topic list
```

### Visualization with RViz2

To visualize the data:

```bash
rviz2
```

Add displays:

1. **Image**: `/camera/color/image_raw`
2. **PointCloud2**: `/camera/depth/color/points`
3. **DepthCloud**: `/camera/depth/image_rect_raw`

---

## Advanced Configuration

### Launch Parameters

The `rs_launch.py` file accepts several parameters:

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_gyro:=true \
    enable_accel:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30
```


## Troubleshooting

### Common Issues

#### Camera Not Detected

```bash

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### OpenGL Compilation Error

If examples cause errors:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false
```

#### Package realsense2_camera Not Found

Always source the workspace:

```bash
source ~/bolide_voiture/install/setup.bash
```
---

## Conclusion

The Intel RealSense D435i camera is now fully functional with ROS2 Jazzy. Depth, RGB, and IMU streams are available for autonomous navigation and 3D SLAM applications.

### Summary of Steps

1. Installation of librealsense from source
2. Configuration of udev rules
3. Compilation without graphical examples
4. Installation of ROS2 wrapper
5. Testing and validation

### Next Steps

- Integration with a SLAM algorithm (ORB-SLAM3, RTAB-Map)
