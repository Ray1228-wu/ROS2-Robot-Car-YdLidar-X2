#!/usr/bin/env python3
"""
WASD 鍵盤遙控（PC 端使用）
發布到 /manual_control (std_msgs/String): W/S/A/D/STOP
支援 M 鍵發布 /motor_mode=toggle，Q 退出。
"""

import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub_manual = self.create_publisher(String, '/manual_control', 10)
        self.pub_mode = self.create_publisher(String, '/motor_mode', 10)
        self.rate_hz = 10.0
        self.release_timeout = 0.3
        self._last_cmd = 'STOP'
        self._last_time = time.time()
        self._orig_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info('W/A/S/D 控制，M 切換模式，Q 離開。')

    def _read_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            ch = sys.stdin.read(1)
            return ch
        return None

    def _publish(self, text: str):
        msg = String()
        msg.data = text
        self.pub_manual.publish(msg)

    def _publish_mode(self, text: str):
        msg = String()
        msg.data = text
        self.pub_mode.publish(msg)

    def spin(self):
        try:
            period = 1.0 / self.rate_hz
            while rclpy.ok():
                ch = self._read_key()
                now = time.time()

                if ch:
                    key = ch.lower()
                    if key == 'q':
                        self.get_logger().info('退出')
                        break
                    elif key == 'm':
                        self._publish_mode('toggle')
                        self.get_logger().info('已發布 motor_mode=toggle')
                    elif key in ['w', 'a', 's', 'd']:
                        self._last_cmd = key.upper()
                        self._last_time = now
                    else:
                        # 其他鍵不處理
                        pass

                if now - self._last_time > self.release_timeout:
                    if self._last_cmd != 'STOP':
                        self._last_cmd = 'STOP'

                self._publish(self._last_cmd)
                time.sleep(period)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._orig_settings)


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleop()
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
