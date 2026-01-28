#!/usr/bin/env python3
"""
Smart Obstacle Avoidance Node for RB5
--------------------------------------

- Subscribes to Intel RealSense depth topic
- Slices the depth image to compute distances
- Calculates speed and steering based on obstacles
- Sends commands via UDP to Raspberry Pi
- Works without cv_bridge or OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
import socket
import json
import time

# CONFIGURATION

class Config:
    # Distance thresholds in centimeters
    OBSTACLE_DETECT = 80.0    # start reacting to obstacles
    MIN_DISTANCE = 30.0       # minimum safe distance

    # Speed limits in meters/second
    MAX_SPEED = 0.06
    MIN_SPEED = 0.02

    # Camera parameters
    HFOV_DEG = 87.0           # horizontal field of view
    NUM_SLICES = 120          # number of horizontal slices to analyze

    # Steering and stopping field of view
    STEER_FOV_DEG = 20.0      # FOV used for steering calculation
    STOP_FOV_DEG = 25.0       # FOV used for minimum distance calculation

    # Floor rejection (camera ~18cm above ground)
    FLOOR_IGNORE_START = 0.15  # start row fraction to ignore (bottom)
    FLOOR_IGNORE_END   = 0.55  # end row fraction to ignore


# UDP NETWORK CONFIG

UDP_IP = "255.255.255.255"  # broadcast address
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


# SMART OBSTACLE AVOIDANCE NODE

class SmartObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('smart_obstacle_avoidance')

        # Subscribe to depth image topic from RealSense
        self.sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info("Smart Obstacle Avoidance node started (RB5)")

    # Convert ROS depth image (16UC1) to numpy array in cm

    def ros_depth_to_numpy(self, msg):
        if msg.encoding != '16UC1':
            self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
            return None

        depth = np.frombuffer(msg.data, dtype=np.uint16)
        depth = depth.reshape(msg.height, msg.width)

        # Convert mm -> cm
        return depth.astype(np.float32) * 0.1

    # Slice image horizontally and compute minimum distance per slice

    def compute_slice_distances(self, depth_cm):
        h, w = depth_cm.shape
        slice_width = w // Config.NUM_SLICES

        # Ignore the floor region
        h_start = int(h * Config.FLOOR_IGNORE_START)
        h_end   = int(h * Config.FLOOR_IGNORE_END)

        distances = []

        for i in range(Config.NUM_SLICES):
            x1 = i * slice_width
            x2 = x1 + slice_width
            roi = depth_cm[h_start:h_end, x1:x2]
            valid = roi[(roi > 0) & (roi < 300)]  # consider only valid depth values
            distances.append(np.min(valid) if valid.size else np.inf)

        return np.array(distances), slice_width, w

    # Extract front slices within a field of view

    def get_front(self, distances, fov_deg):
        slice_deg = Config.HFOV_DEG / Config.NUM_SLICES
        half = int(fov_deg / slice_deg)
        center = Config.NUM_SLICES // 2
        return distances[center - half:center + half]

    # Calculate speed based on minimum front distance

    def calculate_speed(self, front_min):
        if front_min == np.inf or front_min >= Config.OBSTACLE_DETECT:
            return Config.MAX_SPEED

        if front_min <= Config.MIN_DISTANCE:
            return Config.MIN_SPEED

        ratio = (front_min - Config.MIN_DISTANCE) / (Config.OBSTACLE_DETECT - Config.MIN_DISTANCE)
        return Config.MIN_SPEED + ratio * (Config.MAX_SPEED - Config.MIN_SPEED)

    # Calculate steering angle based on slice distances

    def calculate_steering(self, distances, slice_width, image_width):
        valid = np.copy(distances)
        valid[np.isinf(valid)] = 0  # treat infinite distances as zero for steering

        slice_deg = Config.HFOV_DEG / Config.NUM_SLICES
        half = int(Config.STEER_FOV_DEG / slice_deg)
        center = Config.NUM_SLICES // 2

        idx = np.arange(center - half, center + half)
        weights = valid[center - half:center + half]

        if np.max(weights) == 0:
            return 0.0

        weighted = np.average(idx, weights=weights)
        pixel = (weighted + 0.5) * slice_width
        img_center = image_width / 2

        angle = (pixel - img_center) / image_width * Config.HFOV_DEG

        # Small dead zone around center
        if abs(pixel - img_center) < slice_width:
            angle = 0.0

        return round(angle, 1)

    # Depth image callback

    def depth_callback(self, msg):
        depth_cm = self.ros_depth_to_numpy(msg)
        if depth_cm is None:
            return

        slices, slice_width, img_width = self.compute_slice_distances(depth_cm)
        front = self.get_front(slices, Config.STOP_FOV_DEG)

        min_front = np.min(front)
        speed = self.calculate_speed(min_front)

        steering = 0.0
        if min_front < Config.OBSTACLE_DETECT:
            steering = self.calculate_steering(slices, slice_width, img_width)

        # UDP message: Must match Raspberry Pi fields

        telemetry = {
            "velocity": round(speed, 3),  # m/s
            "steer": steering,             # degrees
            "stop": False                  # never stop in UDP; Raspberry ignores stop logic
        }

        # Send UDP packet
        sock.sendto(json.dumps(telemetry).encode(), (UDP_IP, UDP_PORT))

        # Print info for debugging
        self.get_logger().info(
            f"Speed {speed:.3f} m/s | Steering {steering}° | Front {min_front:.1f} cm"
        )


# MAIN

def main(args=None):
    rclpy.init(args=args)
    node = SmartObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
