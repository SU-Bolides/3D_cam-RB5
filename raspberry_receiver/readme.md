# Raspberry Receiver
A ROS2 node that receives vehicle commands via **UDP** and sends them directly to motor controllers using ROS topics.  
Includes a **watchdog mechanism** to ensure emergency stops if UDP packets stop arriving.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [Configuration](#configuration)
- [Troubleshoot](#troubleshoot)
- [License](#license)

---

## Overview

This node acts as a bridge between an external controller (e.g., RB5 robot or remote interface) and motor controllers. It receives JSON commands over UDP and publishes them to ROS topics:

- `/cmd_vel` → Vehicle velocity
- `/cmd_dir` → Steering angle

A watchdog ensures that if communication is lost for a configurable period, the vehicle is immediately stopped.

---

## Features

- **UDP Listener**: Receives JSON-formatted vehicle commands over UDP.
- **ROS2 Publishers**: Sends velocity and steering commands to the motor controllers.
- **Watchdog**: Stops the vehicle automatically if UDP packets stop arriving.
- **Threaded Listener**: Non-blocking UDP reception.
- **Safe Defaults**: Emergency stop on errors or timeouts.

---

## Dependencies

- **Python 3**  
- **ROS2 (Foxy or later)** with the following packages:
  - `rclpy`
  - `std_msgs`
- Built-in Python modules: `socket`, `json`, `threading`, `time`

**Important:** Ensure the following ROS2 nodes are running for full functionality:

- `cmd_dir_node`
- `stm32_node`
- `cmd_vel_node`

These nodes handle the actual motor control.

---

## Dependencies

- **Python 3**  
- **ROS2 (Foxy or later)** with the following packages:
  - `rclpy`
  - `std_msgs`
- Built-in Python modules: `socket`, `json`, `threading`, `time`

**Important:** Ensure the following ROS2 nodes are running for full functionality:

- `cmd_dir_node`
- `stm32_node`
- `cmd_vel_node`

These nodes handle the actual motor control.

---

## Usage

1. Launch the necessary motor nodes:

```bash
ros2 run bolide_direction cmd_dir_node
ros2 run bolide_stm32 stm32_node
ros2 run bolide_stm32 cmd_vel_node
```

2. Launch the Receiver node:

```bash
ros2 run camera raspberry_receiver
```

---

## How It Works

1. **UDP Listener Thread**  
   - Listens on `0.0.0.0:5005` for incoming UDP packets.  
   - Expects JSON with `velocity` and `steer`.  
   - Updates internal state and publishes to ROS topics immediately.

2. **ROS2 Publishers**  
   - `/cmd_vel` → `Float32`  
   - `/cmd_dir` → `Float32`  

3. **Watchdog Mechanism**  
   - Checks for elapsed time since last UDP packet every 0.1s.  
   - If time > 1s, publishes `0.0` to both velocity and steering → emergency stop.

---

## Configuration

Parameters at the top of the script can be adjusted:

```python
UDP_PORT = 5005          # UDP port
WATCHDOG_TIMEOUT = 1.0   # Emergency stop timeout (seconds)
WATCHDOG_RATE = 0.1      # Watchdog check interval (seconds)
```

## Troubleshoot

- **No movement from motors:**
  - Make sure `cmd_dir_node`, `stm32_node`, and `cmd_vel_node` are running.
  - Check ROS2 topics:
    ```bash
    ros2 topic list
    ```
    Ensure `/cmd_vel` and `/cmd_dir` exist.
  
- **UDP commands not received:**
  - Verify your UDP sender is sending to the correct IP and port (`5005`).
  - Check that firewall is not blocking UDP packets.

- **Watchdog triggering immediately:**
  - Ensure UDP packets are arriving faster than `WATCHDOG_TIMEOUT`.
  - Increase `WATCHDOG_TIMEOUT` if network latency is high.

- **JSON decode errors:**
  - Ensure packets are valid JSON and contain numeric `velocity` and `steer`.
