#!/usr/bin/env python3
"""
Smart Obstacle Avoidance Node for RB5 (ROS2)
-------------------------------------------
- Subscribes to Intel RealSense depth topic
- Slices the depth image to compute distances
- Calculates speed and steering based on obstacles
- Publishes directly to ROS2 topics:
    /cmd_vel (Float32) -> velocity in m/s
    /cmd_dir (Float32) -> steering in degrees
- No UDP, no cv_bridge required
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import numpy as np

# CONFIGURATION PARAMETERS
class Config:
    # Distance thresholds in cm
    OBSTACLE_DETECT = 80.0    # start reacting to obstacles
    MIN_DISTANCE = 20.0       # minimum safe distance

    # Speed limits in m/s
    MAX_SPEED = 0.06
    MIN_SPEED = 0.02

    # Camera parameters
    HFOV_DEG = 87.0           # horizontal field of view
    NUM_SLICES = 120          # number of horizontal slices

    # Steering and stopping FOV
    STEER_FOV_DEG = 20.0
    STOP_FOV_DEG = 25.0

    # Floor rejection (camera ~18cm above ground)
    FLOOR_IGNORE_START = 0.20  # fraction from bottom
    FLOOR_IGNORE_END   = 0.55


class SmartObstacleAvoidance(Node):
    """
    ROS2 node for smart obstacle avoidance using a depth camera.
    """

    def __init__(self):
        super().__init__('smart_obstacle_avoidance')

        # Subscribe to depth image from RealSense
        self.sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # ROS2 publishers for direct motor commands
        self.vel_pub = self.create_publisher(Float32, '/cmd_vel', 10)
        self.dir_pub = self.create_publisher(Float32, '/cmd_dir', 10)

        self.get_logger().info("Smart Obstacle Avoidance (ROS2 publishers) started")

    # ---------------- Convert ROS2 depth image to numpy array ----------------
    def ros_depth_to_numpy(self, msg):
        if msg.encoding != '16UC1':
            self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
            return None

        depth = np.frombuffer(msg.data, dtype=np.uint16)
        depth = depth.reshape(msg.height, msg.width)
        return depth.astype(np.float32) * 0.1  # mm -> cm

    # ---------------- Slice image horizontally and compute min distance per slice ----------------
    def compute_slice_distances(self, depth_cm):
        h, w = depth_cm.shape
        slice_width = w // Config.NUM_SLICES

        # Ignore floor region
        h_start = int(h * Config.FLOOR_IGNORE_START)
        h_end   = int(h * Config.FLOOR_IGNORE_END)

        distances = []

        for i in range(Config.NUM_SLICES):
            x1 = i * slice_width
            x2 = x1 + slice_width
            roi = depth_cm[h_start:h_end, x1:x2]
            valid = roi[(roi > 0) & (roi < 300)]  # valid depth values only
            distances.append(np.min(valid) if valid.size else np.inf)

        return np.array(distances), slice_width, w

    # ---------------- Extract front slices within a field of view ----------------
    def get_front(self, distances, fov_deg):
        slice_deg = Config.HFOV_DEG / Config.NUM_SLICES
        half = int(fov_deg / slice_deg)
        center = Config.NUM_SLICES // 2
        return distances[center - half:center + half]

    # ---------------- Calculate forward speed based on front distance ----------------
    def calculate_speed(self, front_min):
        if front_min == np.inf or front_min >= Config.OBSTACLE_DETECT:
            return Config.MAX_SPEED
        if front_min <= Config.MIN_DISTANCE:
            return Config.MIN_SPEED

        ratio = (front_min - Config.MIN_DISTANCE) / (Config.OBSTACLE_DETECT - Config.MIN_DISTANCE)
        return Config.MIN_SPEED + ratio * (Config.MAX_SPEED - Config.MIN_SPEED)

    # ---------------- Calculate steering angle based on slice distances ----------------
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

        # Dead zone around center
        if abs(pixel - img_center) < slice_width:
            angle = 0.0

        return round(angle, 1)

    # ---------------- Depth image callback ----------------
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

        # Publish directly to ROS2 topics
        self.vel_pub.publish(Float32(data=speed))
        self.dir_pub.publish(Float32(data=steering))

        self.get_logger().info(
            f"Speed {speed:.3f} m/s | Steering {steering}° | Front {min_front:.1f} cm"
        )


# ---------------- Main ----------------
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
