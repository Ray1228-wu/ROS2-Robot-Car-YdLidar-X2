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
import os
import sys

# 檢查是否在 test mode（非 Raspberry Pi 環境）
TEST_MODE = os.environ.get('GPIO_TEST_MODE', '0') == '1' or not os.path.exists('/dev/mem')

try:
    if not TEST_MODE:
        from gpiozero import PWMOutputDevice, Motor
except:
    TEST_MODE = True

from pathlib import Path

class ManualMotorDriver(Node):
    def __init__(self):
        super().__init__('manual_motor_driver')
        
        self.test_mode = TEST_MODE
        
        # --- 參數設定 ---
        self.MAX_PWM = 1.0  # 最大 PWM (100% 動力)
        self.MIN_PWM = 0.35  # 最低啟動 PWM
        self.TURN_PWM = 0.7  # 轉向時的 PWM
        self.FORWARD_PWM = 0.8  # 前進時的 PWM

        # 指令超時設定：按鍵鬆開後前端會送 STOP/空字串，但若通訊中斷，超時自動停車
        self.CMD_TIMEOUT = 0.5  # 秒
        self._last_cmd_time = self.get_clock().now()
        self._is_running = False
        
        # --- GPIO 初始化 ---
        if not self.test_mode:
            # 左輪：Pin 18 (PWM), Pin 14 (forward), Pin 15 (backward)
            self.motor_l_pwm = PWMOutputDevice(18, frequency=100)
            self.motor_l_dir = Motor(forward=14, backward=15)
            
            # 右輪：Pin 22 (PWM), Pin 17 (forward), Pin 27 (backward)
            self.motor_r_pwm = PWMOutputDevice(22, frequency=100)
            self.motor_r_dir = Motor(forward=17, backward=27)
        else:
            # Test mode: mock GPIO objects
            self.motor_l_pwm = None
            self.motor_l_dir = None
            self.motor_r_pwm = None
            self.motor_r_dir = None

        # --- ROS 訂閱 ---
        self.sub_manual = self.create_subscription(
            String, '/manual_control', self.manual_control_callback, 10)
        
        self.get_logger().info('✅ 已成功訂閱: /manual_control')
        self.get_logger().info('   等待手動控制指令...')

        # 週期檢查超時，沒有指令就自動停車
        self.timer = self.create_timer(0.1, self._timeout_watchdog)
        
        self.get_logger().info('🎮 手動控制馬達驅動節點啟動')
        self.get_logger().info('   ROS 節點名: manual_motor_driver')
        if self.test_mode:
            self.get_logger().warn('⚠️  Test mode: GPIO 模擬運行')

    def manual_control_callback(self, msg):
        """處理手動控制指令"""
        command = msg.data.strip().upper()
        self._last_cmd_time = self.get_clock().now()
        
        self.get_logger().debug(f'📨 收到指令: {repr(command)}')
        
        # 空指令或 STOP 代表鬆開所有按鍵，立即停車
        if not command or command == 'STOP':
            self._stop_motors()
            return
        
        # 處理前端格式: FORWARD|0.3, BACKWARD|0.3, LEFT|0.3, RIGHT|0.3
        if '|' in command:
            # 解析 "COMMAND|SPEED" 格式
            parts = command.split('|')
            cmd_type = parts[0]
            speed = float(parts[1]) if len(parts) > 1 else self.FORWARD_PWM
            
            # 更新 PWM 值（可選：動態調整速度）
            # 這裡暫時忽略速度參數，使用預設值
            if cmd_type == 'FORWARD':
                self._move_forward()
            elif cmd_type == 'BACKWARD':
                self._move_backward()
            elif cmd_type == 'LEFT':
                self._turn_left()
            elif cmd_type == 'RIGHT':
                self._turn_right()
            else:
                self._stop_motors()
        else:
            # 向後兼容：處理單字符指令 W/S/A/D
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

    def _timeout_watchdog(self):
        """若超過 CMD_TIMEOUT 沒收到指令，自動停車"""
        now = self.get_clock().now()
        if (now - self._last_cmd_time).nanoseconds > self.CMD_TIMEOUT * 1e9:
            if self._is_running:
                self._stop_motors()
                self._is_running = False
        else:
            # 有指令時標記為運行狀態
            self._is_running = True

    def _move_forward(self):
        """前進：左右輪同向前進"""
        if not self.test_mode:
            self.motor_l_dir.forward()
            self.motor_r_dir.forward()
            self.motor_l_pwm.value = self.FORWARD_PWM
            self.motor_r_pwm.value = self.FORWARD_PWM
        self.get_logger().info('⬆️  前進', throttle_duration_sec=1.0)

    def _move_backward(self):
        """後退：左右輪同向後退"""
        if not self.test_mode:
            self.motor_l_dir.backward()
            self.motor_r_dir.backward()
            self.motor_l_pwm.value = self.FORWARD_PWM
            self.motor_r_pwm.value = self.FORWARD_PWM
        self.get_logger().info('⬇️  後退', throttle_duration_sec=1.0)

    def _turn_left(self):
        """左轉：原地旋轉，左輪後退，右輪前進"""
        if not self.test_mode:
            self.motor_l_dir.backward()
            self.motor_r_dir.forward()
            self.motor_l_pwm.value = self.TURN_PWM
            self.motor_r_pwm.value = self.TURN_PWM
        self.get_logger().info('⬅️  左轉', throttle_duration_sec=1.0)

    def _turn_right(self):
        """右轉：原地旋轉，右輪後退，左輪前進"""
        if not self.test_mode:
            self.motor_l_dir.forward()
            self.motor_r_dir.backward()
            self.motor_l_pwm.value = self.TURN_PWM
            self.motor_r_pwm.value = self.TURN_PWM
        self.get_logger().info('➡️  右轉', throttle_duration_sec=1.0)

    def _stop_motors(self):
        """停止馬達"""
        if not self.test_mode:
            self.motor_l_dir.stop()
            self.motor_r_dir.stop()
            self.motor_l_pwm.value = 0
            self.motor_r_pwm.value = 0
        self.get_logger().info('⏸️  暫停', throttle_duration_sec=1.0)

    def stop_motors_safe(self):
        """安全停止馬達"""
        try:
            if not self.test_mode:
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
    
    # 設置日誌級別為 INFO 確保訂閱訊息被顯示
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    
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
