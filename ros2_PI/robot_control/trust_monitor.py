#!/usr/bin/env python3
"""
Trust / Safety Monitor
- Detect SLAM pose jumps > 0.2 m between consecutive SLAM updates
- Check odom vs SLAM consistency: if odom delta >= 0.10 m while SLAM delta <= 0.02 m
- On anomaly, publish STOP to /manual_control (std_msgs/String)

Topics (defaults):
- SLAM pose: /slam_pose (geometry_msgs/PoseStamped)
- Odom: /odometry/local (nav_msgs/Odometry)
- STOP output: /manual_control (std_msgs/String)

Cooldown: once every 2s per anomaly type
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


def _dist(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


class TrustMonitor(Node):
    def __init__(self):
        super().__init__('trust_monitor')

        # Parameters (could be extended to declare parameters later)
        self.slam_topic = '/slam_pose'
        self.odom_topic = '/odometry/local'
        self.jump_threshold = 0.2      # meters
        self.odom_big = 0.10           # odom delta threshold
        self.slam_small = 0.02         # slam delta threshold
        self.cooldown_sec = 2.0

        # State
        self.last_slam = None        # (x, y, stamp)
        self.prev_slam = None        # previous slam point
        self.last_odom = None        # (x, y, stamp)
        self.last_stop_time = {'jump': 0.0, 'mismatch': 0.0}

        # Publishers / Subscribers
        self.stop_pub = self.create_publisher(String, '/manual_control', 10)
        self.sub_slam = self.create_subscription(PoseStamped, self.slam_topic, self.on_slam, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.get_logger().info('Trust monitor started: watching SLAM jumps and odom/SLAM mismatch')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _send_stop(self, reason: str):
        now = self._now()
        # cooldown per reason
        if now - self.last_stop_time[reason] < self.cooldown_sec:
            return
        self.last_stop_time[reason] = now
        msg = String()
        msg.data = 'STOP'
        self.stop_pub.publish(msg)
        self.get_logger().warn(f'EMERGENCY STOP ({reason}) -> publish STOP on /manual_control')

    def on_slam(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_slam is not None:
            dist = _dist((x, y), (self.last_slam[0], self.last_slam[1]))
            if dist > self.jump_threshold:
                self.get_logger().warn(f'SLAM jump detected: {dist:.3f} m')
                self._send_stop('jump')

        # keep previous for delta computation
        self.prev_slam = self.last_slam
        self.last_slam = (x, y, stamp)

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Compute odom delta vs previous odom
        if self.last_odom is not None:
            odom_delta = _dist((x, y), (self.last_odom[0], self.last_odom[1]))
        else:
            odom_delta = 0.0

        # Store odom history
        self.last_odom = (x, y, stamp)

        # Odom vs SLAM consistency: need two slam samples
        if self.prev_slam is not None and self.last_slam is not None and odom_delta >= self.odom_big:
            slam_delta = _dist((self.last_slam[0], self.last_slam[1]), (self.prev_slam[0], self.prev_slam[1]))
            if slam_delta <= self.slam_small:
                self.get_logger().error(
                    f'Odom/SLAM mismatch: odom_delta={odom_delta:.3f} m, slam_delta={slam_delta:.3f} m'
                )
                self._send_stop('mismatch')

def main(args=None):
    rclpy.init(args=args)
    node = TrustMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
