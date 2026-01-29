# RB5 - Camera Control Suite

This repository contains ROS2 Python nodes for autonomous navigation and motor control of the RB5 robot.  
It is designed to provide **real-time obstacle avoidance** and direct motor commands, using a combination of depth sensing and UDP communication.

---

## Contents

The main folder includes:

1. **`raspberry_receiver.py`**  
   - Receives vehicle commands via UDP (JSON format).  
   - Publishes velocity and steering to ROS2 topics (`/cmd_vel` and `/cmd_dir`).  
   - Includes a watchdog: stops the robot if UDP packets stop arriving.  

2. **`rb5_obstacle_avoidance.py`**   
   - Processes depth images to compute obstacle distances across horizontal slices.  
   - Calculates safe forward speed and steering angles based on detected obstacles.  
   - Sends commands via UDP to the Raspberry Pi, which forwards them to the motor nodes.


This setup allows the RB5 robot to navigate autonomously while reacting safely to obstacles.

---

## Features

- Autonomous obstacle avoidance using depth sensing  
- Dynamic speed and steering adjustment  
- UDP-based communication between perception and motor control  
- Watchdog mechanism for emergency stop  
- Floor rejection to ignore false obstacles near the camera  

