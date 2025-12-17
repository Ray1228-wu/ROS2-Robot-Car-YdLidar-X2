#!/usr/bin/env python3
"""
马达模式管理器 - 在手动/自动模式间切换
订阅 /motor_mode 主题，动态启停 manual_motor_driver 和 motor_driver
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import os
import sys
from pathlib import Path
import time

class MotorModeManager(Node):
    def __init__(self):
        super().__init__('motor_mode_manager')
        
        self.current_mode = 'auto'  # 'auto' 或 'manual'
        self.manual_process = None
        self.auto_process = None
        
        # 获取脚本路径
        self.script_dir = Path(__file__).parent
        self.manual_script = self.script_dir / 'manual_motor_driver.py'
        self.auto_script = self.script_dir / 'motor_driver.py'
        
        # 订阅 PC 端模式状态（由 PC 发布到 /motor_mode）
        self.mode_sub = self.create_subscription(
            String, '/motor_mode', self.mode_callback, 10)

        # 監聽手動指令：當前是 auto 且收到 /manual_control，立即切到 manual
        self.manual_control_sub = self.create_subscription(
            String, '/manual_control', self.manual_control_callback, 10)
        self.last_auto_to_manual_time = 0.0
        self.AUTO_TO_MANUAL_COOLDOWN = 1.0  # 秒，避免抖動
        
        self.get_logger().info('🔄 马达模式管理器启动')
        self.get_logger().info(f'   当前模式: {self.current_mode}')
        
        # 默认启动自动模式
        self.switch_to_auto()
    
    def mode_callback(self, msg):
        """处理模式切换请求"""
        requested_mode = msg.data.strip().lower()
        
        # 支援單鍵切換：收到 'toggle'/'switch' 時在 auto/manual 間互換
        if requested_mode in ['toggle', 'switch', 't']:
            requested_mode = 'manual' if self.current_mode == 'auto' else 'auto'
            self.get_logger().info(f'🔁  觸發切換: {requested_mode}')
        elif requested_mode not in ['auto', 'manual']:
            self.get_logger().warn(f'⚠️  无效模式: {requested_mode}')
            return
        
        if requested_mode == self.current_mode:
            self.get_logger().info(f'ℹ️  已经是 {requested_mode} 模式')
            return
        
        self.get_logger().info(f'🔄 切换模式: {self.current_mode} → {requested_mode}')
        
        if requested_mode == 'manual':
            self.switch_to_manual()
        else:
            self.switch_to_auto()

    def manual_control_callback(self, msg):
        """當前為 auto 時，一旦收到手動指令即切換到 manual。"""
        if self.current_mode != 'auto':
            return
        now = time.time()
        if now - self.last_auto_to_manual_time < self.AUTO_TO_MANUAL_COOLDOWN:
            return
        self.last_auto_to_manual_time = now
        self.get_logger().info('🕹️  偵測到手動控制指令，切換到手動模式')
        self.switch_to_manual()
    
    def switch_to_manual(self):
        """切换到手动模式"""
        # 1. 停止自动驱动
        self.stop_auto_driver()
        
        # 2. 启动手动驱动
        self.start_manual_driver()
        
        self.current_mode = 'manual'
        self.get_logger().info('✅ 已切换到手动模式')
    
    def switch_to_auto(self):
        """切换到自动模式"""
        # 1. 停止手动驱动
        self.stop_manual_driver()
        
        # 2. 启动自动驱动
        self.start_auto_driver()
        
        self.current_mode = 'auto'
        self.get_logger().info('✅ 已切换到自动模式')
    
    def start_manual_driver(self):
        """启动手动马达驱动"""
        if self.manual_process and self.manual_process.poll() is None:
            self.get_logger().warn('⚠️  手动驱动已在运行')
            return
        
        try:
            env = os.environ.copy()
            # 確保子程序可使用 user site 套件（避免從父進程繼承 PYTHONNOUSERSITE=1）
            env.pop('PYTHONNOUSERSITE', None)
            self.manual_process = subprocess.Popen(
                [sys.executable, str(self.manual_script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f'▶️  手动驱动已启动 (PID: {self.manual_process.pid})')
        except Exception as e:
            self.get_logger().error(f'❌ 启动手动驱动失败: {e}')
    
    def start_auto_driver(self):
        """启动自动马达驱动"""
        if self.auto_process and self.auto_process.poll() is None:
            self.get_logger().warn('⚠️  自动驱动已在运行')
            return
        
        try:
            env = os.environ.copy()
            # 確保子程序可使用 user site 套件（避免從父進程繼承 PYTHONNOUSERSITE=1）
            env.pop('PYTHONNOUSERSITE', None)
            self.auto_process = subprocess.Popen(
                [sys.executable, str(self.auto_script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f'▶️  自动驱动已启动 (PID: {self.auto_process.pid})')
        except Exception as e:
            self.get_logger().error(f'❌ 启动自动驱动失败: {e}')
    
    def stop_manual_driver(self):
        """停止手动马达驱动"""
        if self.manual_process and self.manual_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.manual_process.pid), signal.SIGTERM)
                self.manual_process.wait(timeout=2)
                self.get_logger().info('⏹️  手动驱动已停止')
            except Exception as e:
                self.get_logger().warn(f'⚠️  停止手动驱动失败: {e}')
                try:
                    os.killpg(os.getpgid(self.manual_process.pid), signal.SIGKILL)
                except:
                    pass
            self.manual_process = None
    
    def stop_auto_driver(self):
        """停止自动马达驱动"""
        if self.auto_process and self.auto_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.auto_process.pid), signal.SIGTERM)
                self.auto_process.wait(timeout=2)
                self.get_logger().info('⏹️  自动驱动已停止')
            except Exception as e:
                self.get_logger().warn(f'⚠️  停止自动驱动失败: {e}')
                try:
                    os.killpg(os.getpgid(self.auto_process.pid), signal.SIGKILL)
                except:
                    pass
            self.auto_process = None
    
    def cleanup(self):
        """清理所有子进程"""
        self.get_logger().info('🛑 清理子进程...')
        self.stop_manual_driver()
        self.stop_auto_driver()

def main(args=None):
    rclpy.init(args=args)
    manager = MotorModeManager()
    
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info('收到中断信号')
    finally:
        manager.cleanup()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
