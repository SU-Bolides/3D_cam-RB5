#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import socket
import json
import threading
import time

from std_msgs.msg import Float32


# CONFIGURATION

UDP_PORT = 5005          # Port to receive UDP packets from RB5

# Watchdog timeout in seconds:
# If no UDP packet is received for this time → EMERGENCY STOP
WATCHDOG_TIMEOUT = 1.0

# Watchdog check rate (seconds)
WATCHDOG_RATE = 0.1      # 10 Hz


# UDP TO MOTORS NODE

class UdpToMotorsNode(Node):
    """
    ROS2 node that:
    - Receives vehicle commands via UDP (JSON)
    - Sends commands directly to motor controllers via ROS topics
    - Applies a watchdog to stop motors if UDP packets stop arriving
    """

    def __init__(self):
        super().__init__('udp_to_motors_node')

        # -------------------- ROS Publishers --------------------
        self.vel_pub = self.create_publisher(Float32, '/cmd_vel', 10)
        self.dir_pub = self.create_publisher(Float32, '/cmd_dir', 10)

        # -------------------- UDP Setup ------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', UDP_PORT))  # Listen on all interfaces

        self.get_logger().info(f"Listening for UDP packets on port {UDP_PORT}")

        # -------------------- Internal state -------------------
        self.last_msg_time = time.time()
        self.last_velocity = 0.0
        self.last_steer = 0.0

        # -------------------- UDP listener thread --------------
        self.udp_thread = threading.Thread(
            target=self.udp_listener,
            daemon=True
        )
        self.udp_thread.start()

        # -------------------- Watchdog timer -------------------
        self.watchdog_timer = self.create_timer(
            WATCHDOG_RATE,
            self.watchdog_callback
        )

    # UDP Listener

    def udp_listener(self):
        """
        Blocking UDP receiver running in a separate thread.
        Updates the latest command and publishes immediately.
        """
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))

                # ---------------- Parse JSON fields ----------------
                # Only read velocity and steering, ignore "stop"
                self.last_velocity = float(msg.get('velocity', 0.0))
                self.last_steer = float(msg.get('steer', 0.0))

                # Update timestamp for watchdog
                self.last_msg_time = time.time()

                # Publish motor commands immediately
                self.publish_motor_commands()

            except Exception as e:
                self.get_logger().warn(f"UDP receive error: {e}")

    # Watchdog callback

    def watchdog_callback(self):
        """
        Checks if UDP packets are still arriving.
        If timeout exceeded, stops motors immediately.
        """
        elapsed = time.time() - self.last_msg_time
        if elapsed > WATCHDOG_TIMEOUT:
            self.publish_stop()
            self.get_logger().warn(
                f"Watchdog timeout ({elapsed:.2f}s) -> EMERGENCY STOP"
            )

    # Publish motor commands

    def publish_motor_commands(self):
        """
        Publish velocity and steering received via UDP.
        No stop flag is used anymore; always send values.
        """
        vel = self.last_velocity
        steer = self.last_steer

        self.vel_pub.publish(Float32(data=vel))
        self.dir_pub.publish(Float32(data=steer))

        self.get_logger().info(
            f"Command sent | Velocity: {vel:.3f} | Steering: {steer:.1f}"
        )

    # Emergency stop

    def publish_stop(self):
        """
        Immediately stop the vehicle.
        Called by watchdog if UDP packets stop arriving.
        """
        self.vel_pub.publish(Float32(data=0.0))
        self.dir_pub.publish(Float32(data=0.0))


# MAIN

def main(args=None):
    rclpy.init(args=args)
    node = UdpToMotorsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
