#!/usr/bin/env python3
"""
手動控制馬達驅動 - 訂閱 /manual_control 主題
支持 WASD 鍵盤控制 + 空格暫停

- W: 前進
- S: 後退
- A: 左轉
- D: 右轉
- 空格/其他: 暫停
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import PWMOutputDevice, Motor
from pathlib import Path

class ManualMotorDriver(Node):
    def __init__(self):
        super().__init__('manual_motor_driver')
        
        # --- 參數設定 ---
        self.MAX_PWM = 1.0  # 最大 PWM (100% 動力)
        self.MIN_PWM = 0.35  # 最低啟動 PWM
        self.TURN_PWM = 0.7  # 轉向時的 PWM
        self.FORWARD_PWM = 0.8  # 前進時的 PWM
        
        # --- GPIO 初始化 ---
        # 左輪：Pin 18 (PWM), Pin 14 (forward), Pin 15 (backward)
        self.motor_l_pwm = PWMOutputDevice(18, frequency=100)
        self.motor_l_dir = Motor(forward=14, backward=15)
        
        # 右輪：Pin 22 (PWM), Pin 17 (forward), Pin 27 (backward)
        self.motor_r_pwm = PWMOutputDevice(22, frequency=100)
        self.motor_r_dir = Motor(forward=17, backward=27)

        # --- ROS 訂閱 ---
        self.sub_manual = self.create_subscription(
            String, '/manual_control', self.manual_control_callback, 10)
        
        self.get_logger().info('🎮 手動控制馬達驅動節點啟動')
        self.get_logger().info('   訂閱主題: /manual_control')
        self.get_logger().info('   支持指令: W(前進) S(後退) A(左轉) D(右轉) SPACE(暫停)')

    def manual_control_callback(self, msg):
        """處理手動控制指令"""
        command = msg.data.strip().upper()
        
        if not command:
            self._stop_motors()
            return
        
        # 處理第一個字符的指令
        cmd_char = command[0] if command else ' '
        
        if cmd_char == 'W':
            self._move_forward()
        elif cmd_char == 'S':
            self._move_backward()
        elif cmd_char == 'A':
            self._turn_left()
        elif cmd_char == 'D':
            self._turn_right()
        else:  # 空格或其他
            self._stop_motors()

    def _move_forward(self):
        """前進：左右輪同向前進"""
        self.motor_l_dir.forward()
        self.motor_r_dir.forward()
        self.motor_l_pwm.value = self.FORWARD_PWM
        self.motor_r_pwm.value = self.FORWARD_PWM
        self.get_logger().info('⬆️  前進', throttle_duration_sec=1.0)

    def _move_backward(self):
        """後退：左右輪同向後退"""
        self.motor_l_dir.backward()
        self.motor_r_dir.backward()
        self.motor_l_pwm.value = self.FORWARD_PWM
        self.motor_r_pwm.value = self.FORWARD_PWM
        self.get_logger().info('⬇️  後退', throttle_duration_sec=1.0)

    def _turn_left(self):
        """左轉：原地旋轉，左輪後退，右輪前進"""
        self.motor_l_dir.backward()
        self.motor_r_dir.forward()
        self.motor_l_pwm.value = self.TURN_PWM
        self.motor_r_pwm.value = self.TURN_PWM
        self.get_logger().info('⬅️  左轉', throttle_duration_sec=1.0)

    def _turn_right(self):
        """右轉：原地旋轉，右輪後退，左輪前進"""
        self.motor_l_dir.forward()
        self.motor_r_dir.backward()
        self.motor_l_pwm.value = self.TURN_PWM
        self.motor_r_pwm.value = self.TURN_PWM
        self.get_logger().info('➡️  右轉', throttle_duration_sec=1.0)

    def _stop_motors(self):
        """停止馬達"""
        self.motor_l_dir.stop()
        self.motor_r_dir.stop()
        self.motor_l_pwm.value = 0
        self.motor_r_pwm.value = 0
        self.get_logger().info('⏸️  暫停', throttle_duration_sec=1.0)

    def stop_motors_safe(self):
        """安全停止馬達"""
        try:
            self._stop_motors()
            self.motor_l_pwm.close()
            self.motor_l_dir.close()
            self.motor_r_pwm.close()
            self.motor_r_dir.close()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ManualMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中斷信號，關閉馬達...')
        node.stop_motors_safe()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
