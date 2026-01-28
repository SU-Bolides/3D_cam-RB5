#!/usr/bin/env python3
"""
Master Navigator (Camera + Lidar)

Camera:
- Provides long-range direction and speed.

Lidar:
- Used ONLY for emergency stop and short-range avoidance.
- Has priority over camera when danger is close.

Behavior:
- Very fast steering response
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np
import socket
import json
import time

# ===================== UDP CONFIG =====================
UDP_IP = "10.42.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ===================== SPEED PARAMETERS =====================
MAX_SPEED = 0.06    
MIN_SPEED = 0.02    

# ===================== STEERING LIMITS =====================
MAX_STEER_DEG = 22.0   # Physical steering limit (degrees)

# ===================== LIDAR PARAMETERS =====================
LIDAR_MAX_DIST = 0.40 # meters
LIDAR_FRONT_STOP = 0.25 # Emergency stop distance
LIDAR_FRONT_CLEAR = 0.30 # Clear path distance

# ===================== CAMERA PARAMETERS =====================
CAM_MIN_DIST = 0.25 # meters
CAM_MAX_DIST = 3.0 # meters
NUM_SLICES = 120 # Number of image slices for processing
FLOOR_START = 0.15 # Portion of image height for floor ROI
FLOOR_END = 0.55 # Portion of image height for floor ROI

DATA_TIMEOUT = 0.3  # seconds

# ===================== AGGRESSIVE TUNING =====================
STEER_GAIN = 3.5      # HIGH steering responsiveness
ALPHA = 0.8           # Very light smoothing (high = fast response)
# ============================================================


class MasterNavigator(Node):

    def __init__(self):
        super().__init__("master_navigator")

        self.create_subscription(LaserScan, "/scan", self.lidar_cb, 10)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self.cam_cb, 10)

        self.lidar_data = None
        self.lidar_time = 0.0
        self.cam_data = None
        self.cam_time = 0.0

        self.prev_steer = 0.0
        self.prev_vel = 0.0

        self.get_logger().info("Master Navigator - Fusion node started")

    # ---------------- SPEED SCALING ----------------
    def scale_speed(self, dist):
        """Scale forward velocity based on camera distance"""
        if dist <= CAM_MIN_DIST:
            return 0.0
        if dist >= CAM_MAX_DIST:
            return MAX_SPEED

        ratio = (dist - CAM_MIN_DIST) / (CAM_MAX_DIST - CAM_MIN_DIST)
        return MIN_SPEED + (ratio ** 1.5) * (MAX_SPEED - MIN_SPEED)

    # ---------------- LIDAR CALLBACK ----------------
    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.degrees(np.linspace(msg.angle_min, msg.angle_max, len(ranges)))

        # Ignore back hemisphere, invalid and far readings
        mask = (np.abs(angles) > 90) & (ranges > 0.05) & (ranges < LIDAR_MAX_DIST)
        ranges = ranges[mask]
        angles = angles[mask]

        if len(ranges) == 0:
            self.lidar_data = None
            return

        # Front danger zone
        front = ranges[np.abs(angles) > 160]

        # Emergency stop
        if len(front) > 0 and np.min(front) < LIDAR_FRONT_STOP:
            self.lidar_data = ("LIDAR", 0.0, 0.0, "FRONT_STOP")
            self.lidar_time = time.time()
            return

        # Front clear, only side avoidance allowed
        if len(front) > 0 and np.min(front) > LIDAR_FRONT_CLEAR:
            self.lidar_data = ("LIDAR", 0.8, 0.0, "SIDE_ONLY")
            self.lidar_time = time.time()
            return

        # Obstacle avoidance steering
        weights = (LIDAR_MAX_DIST - ranges) / LIDAR_MAX_DIST
        weights = weights ** 2

        steer = np.sum(weights * (-angles)) / np.sum(weights)
        steer = np.clip(steer, -MAX_STEER_DEG, MAX_STEER_DEG)
        steer_norm = steer / MAX_STEER_DEG

        self.lidar_data = ("LIDAR", 1.0, steer_norm, "AVOID")
        self.lidar_time = time.time()

    # ---------------- CAMERA CALLBACK ----------------
    def cam_cb(self, msg):

        # Decode depth image
        if msg.encoding in ["16UC1", "mono16"]:
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width
            ).astype(np.float32) * 0.001
        elif msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )
        else:
            return

        # Select floor region
        h1 = int(msg.height * FLOOR_START)
        h2 = int(msg.height * FLOOR_END)
        slice_w = msg.width // NUM_SLICES

        distances = []
        for i in range(NUM_SLICES):
            roi = depth[h1:h2, i * slice_w:(i + 1) * slice_w]
            valid = roi[(roi > CAM_MIN_DIST) & (roi < CAM_MAX_DIST)]
            distances.append(np.mean(valid) if valid.size else 0.0)

        distances = np.array(distances)

        # Vision-based steering (center of free space)
        weights = distances
        indices = np.arange(NUM_SLICES)
        center = np.sum(weights * indices) / (np.sum(weights) + 1e-6)
        offset = center - NUM_SLICES // 2

        steer_norm = offset / (NUM_SLICES // 2)
        steer_norm = np.clip(steer_norm, -1.0, 1.0)

        # Non-linear steering: more aggressive for large errors
        steer_norm = np.sign(steer_norm) * abs(steer_norm) ** 0.6

        # Speed from front distance
        front_avg = np.mean(distances[NUM_SLICES//2 - 5:NUM_SLICES//2 + 5])
        vel = self.scale_speed(front_avg)

        self.cam_data = ("CAMERA", vel, steer_norm, "LONG_RANGE")
        self.cam_time = time.time()

        self.publish_udp()

    # ---------------- DECISION + UDP OUTPUT ----------------
    def publish_udp(self):
        now = time.time()
        sources = []

        cam_vel, cam_steer = 0.0, 0.0
        if self.cam_data and (now - self.cam_time) < DATA_TIMEOUT:
            _, cam_vel, cam_steer, _ = self.cam_data
            sources.append("CAMERA")

        lidar_steer = 0.0
        lidar_vel_limit = 1.0

        if self.lidar_data and (now - self.lidar_time) < DATA_TIMEOUT:
            _, vel_limit, steer_norm, mode = self.lidar_data
            lidar_steer = steer_norm
            lidar_vel_limit = vel_limit
            sources.append(f"LIDAR({mode})")

        # Combine steering
        steer = cam_steer + lidar_steer
        steer = np.clip(steer, -1.0, 1.0)

        # Steering gain
        steer *= STEER_GAIN
        steer = np.clip(steer, -1.0, 1.0)

        # Apply velocity limit from lidar
        vel = cam_vel * lidar_vel_limit

        # Minimal smoothing (FAST response)
        steer = ALPHA * steer + (1 - ALPHA) * self.prev_steer
        vel   = ALPHA * vel   + (1 - ALPHA) * self.prev_vel

        self.prev_steer = steer
        self.prev_vel = vel

        telemetry = {
            "velocity": float(vel),
            "steer": float(steer)
        }

        sock.sendto(json.dumps(telemetry).encode(), (UDP_IP, UDP_PORT))

        self.get_logger().info(
            f"VEL {vel:.3f} | STEER {steer:.2f} | SOURCE: {' + '.join(sources)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MasterNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
