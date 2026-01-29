# RB5 Navigation Package (Camera + LiDAR Fusion)

ROS2 package for **autonomous navigation of the RB5 robot** using:
- Intel RealSense depth camera
- 2D LiDAR
- UDP communication with Raspberry Pi motor controller

The package contains three navigation nodes, from simple vision-based
obstacle avoidance to full sensor fusion with emergency override.

---

## Nodes Overview

| Node | Description |
|------|------------|
| `camera_avoidance` | Camera-only obstacle avoidance |
| `camera_lidar_avoidance` | Camera + LiDAR fusion |
| `fusion_emergency_stop` | Fusion + STOP / REVERSE override |

Only **one node should be active at a time**.

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

# 1. camera_avoidance

## What it does

This node performs **basic obstacle avoidance using only the depth camera**.

It is designed as:
- simple
- lightweight
- no LiDAR
- fast to test

It directly computes speed and steering from the depth image and sends them via UDP.

---

## How it works (conceptually)

1. **Depth image conversion**
   - Reads `/camera/depth/image_rect_raw`
   - Converts 16UC1 depth values to centimeters

2. **Floor rejection**
   - Ignores bottom part of the image
   - Prevents detecting the floor as obstacle

3. **Horizontal slicing**
   - Image is divided into `NUM_SLICES` vertical columns
   - For each slice → minimum distance is computed

4. **Front region**
   - Only a small forward FOV is used for control
   - This avoids reacting to objects on extreme sides

5. **Speed control**
   - If obstacle is far → max speed
   - If obstacle is close → minimum speed
   - Otherwise → linear interpolation

6. **Steering**
   - Finds the slice with most free space
   - Steers toward that direction

7. **UDP output**
   - Sends velocity and steering to Raspberry Pi

---

## Topics

Subscribes: ```/camera/depth/image_rect_raw```

---

## Run

```bash
ros2 run rb5_navigation camera
```

# 2. camera_lidar_avoidance (Camera + LiDAR Fusion)

## What it does

This node performs **sensor fusion between camera and LiDAR** in order to achieve
safe and stable autonomous navigation.

Each sensor has a clear role:

- **Camera**
  - Long-range perception
  - Detects free space
  - Decides where to go
  - Sets desired speed

- **LiDAR**
  - Short-range perception
  - Emergency stop
  - Obstacle avoidance
  - Limits camera commands when danger is close

LiDAR always has **higher priority** than the camera.

---

## How it works

### Camera pipeline

1. The node subscribes to the depth image:
   ```bash
   /camera/depth/image_rect_raw
   ```
2. The depth image is sliced horizontally into many vertical regions.

3. For each slice, the average valid distance is computed.

4. The algorithm finds the **center of free space** by weighting slices with larger distances.

5. Steering is computed from the offset between:
- image center  
- free-space center

6. Speed is computed from the **average front distance**:
- far → high speed  
- close → low speed  

The camera therefore answers:
> *“Where should I go and how fast?”*

---

### LiDAR pipeline

1. The node subscribes to:
   ```bash
   /scan
   ```
2. Only the **front hemisphere** of the LiDAR is used.

3. Three safety modes exist:

- **Emergency stop**  
  If an obstacle is closer than `LIDAR_FRONT_STOP`  
  → velocity becomes 0

- **Front clear**  
  If the front area is free  
  → camera is allowed, but speed is limited

- **Avoidance mode**  
  If obstacles exist in front or sides  
  → LiDAR generates avoidance steering

4. Steering is computed using weighted angles:
- closer obstacles have more influence
- robot steers away from danger

The LiDAR therefore answers:
> *“Is it safe to continue?”*

---

## Fusion logic

The final command is computed as:

- **Steering**
  ```
  final_steer = camera_steer + lidar_steer
  ```
- **Velocity**
  ```
  final_velocity = camera_velocity × lidar_velocity_limit
  ```
Then:
- steering gain is applied
- small smoothing is applied
- result is sent via UDP

---

## Topics

Subscribes to:
```
/camera/depth/image_rect_raw
/scan
```

---

## UDP Output

```json
{
  "velocity": 0.04,
  "steer": -0.3
}
```


## Run
```ros2 run rb5_navigation fusion
```

# 3. fusion_emergency_stop (Fusion + Emergency Override)

## What it does

This node extends the functionality of `master_navigator`
by adding an **external control layer**.

It allows another system (FSM, safety node, human operator)
to **override all sensor decisions**.

This node becomes the **single authority for motor commands**.

---

## Why it exists

In real robotic systems, autonomy must always be overridable.

Examples:
- A human wants to stop the robot.
- A high-level planner detects a dead-end.
- A safety system detects unstable behavior.

This node solves this by listening to a control topic.

---

## How it works

1. The node runs the **full camera + LiDAR fusion pipeline**.

2. Before sending UDP commands, it checks:
   ``` /drive_override ```

3. Three modes exist:

- **STOP**  
  Robot immediately stops.  
  Sensors are ignored.

- **REVERSE**  
  Robot moves backward at fixed speed.  
  Steering is forced to zero.

- **NORMAL**  
  Robot uses camera + LiDAR fusion.

4. Override always has **absolute priority**.

---

## Topics

Subscribes to:
```
/camera/depth/image_rect_raw
/scan
/drive_override (std_msgs/String)
```

## Run
```
ros2 run rb5_navigation fusion_emergency
```


# System Architecture

The navigation system is built around a **central fusion node**
that combines perception and control.

```
RealSense Depth --->|
|--> Master Navigator --> UDP --> Raspberry Pi --> Motors
LiDAR Scan --------->|

Optional:
High-level controller --> /drive_override
```

### Explanation

- The **camera** provides global navigation information.
- The **LiDAR** provides local safety.
- The **Master Navigator** fuses both.
- The **Raspberry Pi** is responsible only for low-level motor control.
- High-level logic (FSM, safety, human control) can override everything.

Only **one node** is allowed to send motor commands.

---

## Dependencies

### System

- Ubuntu 20.04 / 22.04
- ROS2 Foxy / Humble
- Python 3

### ROS2 packages

- `rclpy`
- `sensor_msgs`
- `std_msgs`

### Python modules

- `numpy`
- `socket`
- `json`
- `time`

### Sensors

- Intel RealSense depth camera
- 2D LiDAR

### RealSense driver

```
ros2 launch realsense2_camera rs_launch.py
```

# UDP Communication

All navigation nodes communicate with the motor controller
using **UDP packets in JSON format**.

Default configuration:

```python
UDP_IP = "10.42.0.1"
UDP_PORT = 5005
```

**Message Format**
```json
{
  "velocity": 0.04,
  "steer": -0.3
}
```
Network assumptions

Robot computer and Raspberry Pi are on the same network.

Raspberry Pi listens on UDP_PORT.

No feedback channel (one-way communication).

Navigation node is the single source of truth.

UDP is used because:

low latency

simple

no ROS needed on Raspberry Pi

# Troubleshooting & Tuning Guide

This section explains how the main parameters affect the robot’s behavior
and how to debug common problems.

---

## Robot oscillates left-right

### Cause
Steering response is too aggressive.

### Parameters involved
```python
STEER_GAIN
ALPHA
NUM_SLICES
```

### Fix
1. Reduce STEER_GAIN
2. Increase ALPHA
3. Reduce NUM_SLICES

### Effect:
1. Lower gain → smoother turns
2. Higher alpha → more filtering
3. Fewer slices → less noise

## Robot reacts to the floor 

### Parameters involved
```python
FLOOR_START
FLOOR_END
```

### Cause 
The camera sees the floor as an obstacle.

## No data from RealSense:
  - Ensure the RealSense camera node is running:
    ```bash
    ros2 launch realsense2_camera rs_launch.py
    ```
  - Check that `/camera/depth/image_rect_raw` exists:
    ```bash
    ros2 topic list
    ```

## UDP commands not received on Raspberry Pi:
  - Verify the Raspberry Pi is on the same network and listening on port `5005`.  
  - Check firewall settings and network broadcast permissions.

## Depth image errors (`Unsupported encoding`)
  - The node only supports `16UC1` depth images.  
  - Confirm RealSense publishes in the correct format.

## Obstacle avoidance too sensitive or too slow**
  - Adjust `OBSTACLE_DETECT`, `MIN_DISTANCE`, `MAX_SPEED`, and `MIN_SPEED` in the `Config` class.





