# Smart Obstacle Avoidance Node for RB5

A ROS2 node that performs **real-time obstacle avoidance** for the RB5 robot using Intel RealSense depth data.  
The node slices the depth image, calculates distances, determines safe speed and steering, and sends commands via UDP to a Raspberry Pi for motor control.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [Configuration](#configuration)
- [Troubleshoot](#troubleshoot)
- [License](#license)

---

## Overview

This node enables RB5 to navigate autonomously by avoiding obstacles detected through its Intel RealSense depth camera.  

**Key responsibilities:**

1. Subscribe to the depth image topic from RealSense.
2. Convert depth images from ROS messages to numpy arrays (cm).
3. Slice the image horizontally to compute minimum distances per slice.
4. Calculate speed and steering based on obstacle proximity.
5. Send real-time commands to the Raspberry Pi via UDP.

The node is fully Python-based and does **not require OpenCV or cv_bridge**, making it lightweight and efficient.

---

## Features

- **Depth Image Processing**: Efficiently slices the depth image to evaluate obstacle distances.  
- **Speed Control**: Adjusts forward speed dynamically according to obstacle proximity.  
- **Steering Control**: Computes steering angle to avoid obstacles, focusing on a limited forward field of view.  
- **UDP Communication**: Sends `velocity`, `steer`, and `stop` flags to Raspberry Pi in JSON format.  
- **Floor Rejection**: Ignores the bottom part of the image to prevent detecting the floor as an obstacle.  
- **Real-time Logging**: Provides speed, steering, and minimum distance info for debugging.  

---

## Dependencies

- **Python 3**  
- **ROS2 (Foxy or later)**  
- ROS2 packages:
  - `rclpy`
  - `sensor_msgs`
- Python built-in modules:
  - `numpy`
  - `socket`
  - `json`
  - `time`

**Important:** Make sure the Intel RealSense camera node is running:

```bash
ros2 launch realsense2_camera rs_launch.py
```

---

## Usage

1. Start the RealSense camera node:

```bash
ros2 launch realsense2_camera rs_launch.py
```

2. Run the Smart Obstacle Avoidance node:

```bash
ros2 run camera smart_obstacle_avoidance
```

3. The node continuously sends UDP commands to the broadcast address 255.255.255.255:5005.
Commands include:

```bash
{
  "velocity": 0.06,  // m/s
  "steer": 5.0,      // degrees
  "stop": false      // placeholder, ignored by Raspberry Pi
}
```

4. Check the console log for speed, steering, and closest obstacle info:

```bash
Speed 0.045 m/s | Steering 5.0° | Front 42.3 cm
```

---


## How It Works

1. **Depth Conversion**  
   - Converts `16UC1` ROS depth images to numpy arrays in centimeters.  
   - Rejects invalid depth values (`0` or >300 cm).

2. **Horizontal Slicing**  
   - Image is divided into `NUM_SLICES` vertical slices.  
   - Only the middle rows are analyzed to ignore the floor.  
   - Minimum distance per slice is computed.

3. **Front Slice Extraction**  
   - Defines a "forward field of view" for obstacle detection (`STOP_FOV_DEG`) and for steering (`STEER_FOV_DEG`).  
   - Only slices within these FOVs are considered for control decisions.

4. **Speed Calculation**  
   - If no obstacle detected (`> OBSTACLE_DETECT` cm), max speed is used.  
   - If obstacle closer than `MIN_DISTANCE`, minimal speed is enforced.  
   - Otherwise, speed is interpolated proportionally between min and max.

5. **Steering Calculation**  
   - Computes a weighted average of distances in the steering FOV.  
   - Converts the weighted slice to an angle relative to the camera center.  
   - Small dead zone around center prevents unnecessary micro-adjustments.

6. **UDP Transmission**  
   - Commands are packaged as JSON and broadcasted to `UDP_IP:UDP_PORT`.  
   - Raspberry Pi receives and executes motor commands.

---

## Configuration

Adjust parameters at the top of the script under the `Config` class:

```python
OBSTACLE_DETECT = 80.0    # cm, distance to start reacting
MIN_DISTANCE = 30.0       # cm, minimum safe distance
MAX_SPEED = 0.06          # m/s
MIN_SPEED = 0.02          # m/s
HFOV_DEG = 87.0           # camera horizontal FOV
NUM_SLICES = 120          # number of horizontal slices
STEER_FOV_DEG = 20.0      # FOV for steering calculation
STOP_FOV_DEG = 25.0       # FOV for minimum distance
FLOOR_IGNORE_START = 0.15 # start fraction to ignore
FLOOR_IGNORE_END = 0.55   # end fraction to ignore
```

You can tune these values to optimize obstacle avoidance for different environments, robot speeds, or camera mounting heights.

---

## Troubleshoot

- **No data from RealSense:**
  - Ensure the RealSense camera node is running:
    ```bash
    ros2 launch realsense2_camera rs_launch.py
    ```
  - Check that `/camera/depth/image_rect_raw` exists:
    ```bash
    ros2 topic list
    ```

- **UDP commands not received on Raspberry Pi:**
  - Verify the Raspberry Pi is on the same network and listening on port `5005`.  
  - Check firewall settings and network broadcast permissions.

- **Depth image errors (`Unsupported encoding`)**
  - The node only supports `16UC1` depth images.  
  - Confirm RealSense publishes in the correct format.

- **Obstacle avoidance too sensitive or too slow**
  - Adjust `OBSTACLE_DETECT`, `MIN_DISTANCE`, `MAX_SPEED`, and `MIN_SPEED` in the `Config` class.

- **Steering oscillations**
  - Check `STEER_FOV_DEG` and slice configuration.  
  - Consider increasing the dead-zone by adjusting slice averaging.
