#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
導航系統測試腳本
驗證 ROS 2 話題和導航功能是否正確配置

使用方法:
  1. 啟動 start_all.py: python3 start_all.py
  2. 在另一個終端運行: python3 test_navigation.py
  3. 觀察輸出驗證話題通訊
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import json

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 【接收】訂閱導航目標
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/nav_goal',
            self.goal_callback,
            qos_profile
        )
        
        # 【接收】訂閱馬達控制信號
        self.motor_sub = self.create_subscription(
            String,
            '/motor_control',
            self.motor_callback,
            qos_profile
        )
        
        # 【發送】發佈編碼器狀態（模擬）
        self.encoder_pub = self.create_publisher(
            Twist,
            '/encoder_state',
            qos_profile
        )
        
        # 【發送】發佈機器人狀態
        self.status_pub = self.create_publisher(
            String,
            '/robot_status',
            qos_profile
        )
        
        self.get_logger().info("=== 導航系統測試 ===")
        self.get_logger().info("監聽話題: /nav_goal, /motor_control")
        self.get_logger().info("發送話題: /encoder_state, /robot_status")
        self.get_logger().info("等待來自網頁的導航命令...")
        
        # 定時發送模擬編碼器數據
        self.timer = self.create_timer(0.5, self.publish_encoder_state)
        self.encoder_x = 0.0
        self.encoder_y = 0.0
        
    def goal_callback(self, msg):
        """接收來自網頁的導航目標"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.get_logger().info(f"✓ 收到導航目標: ({goal_x:.2f}, {goal_y:.2f})")
        
        # 發送狀態更新
        status_msg = String()
        status_msg.data = f"導航中 → ({goal_x:.2f}, {goal_y:.2f})"
        self.status_pub.publish(status_msg)
        
    def motor_callback(self, msg):
        """接收馬達控制信號"""
        command = msg.data
        self.get_logger().info(f"✓ 接收馬達命令: {command}")
        
        if command == "PAUSE":
            self.get_logger().warn("⏸ 馬達暫停")
        elif command == "RESUME":
            self.get_logger().info("▶ 馬達繼續")
        elif command == "CANCEL":
            self.get_logger().error("✕ 導航已取消")
            
        # 發送狀態
        status_msg = String()
        status_msg.data = f"馬達狀態: {command}"
        self.status_pub.publish(status_msg)
        
    def publish_encoder_state(self):
        """發送模擬編碼器數據"""
        # 模擬緩慢移動
        self.encoder_x += 0.01
        self.encoder_y += 0.005
        
        msg = Twist()
        msg.linear.x = self.encoder_x
        msg.linear.y = self.encoder_y
        msg.angular.z = 0.0
        
        self.encoder_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n=== 測試終止 ===")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
