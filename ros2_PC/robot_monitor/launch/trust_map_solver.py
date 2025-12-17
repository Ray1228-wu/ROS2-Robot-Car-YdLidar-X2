#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trust Map Solver - 處理 YDLidar X2 掃描數據並轉換為網頁可視化地圖

訂閱話題:
  /scan_throttled - YDLidar X2 已節流掃描數據

發佈話題:
  /viz_map_data - 網頁可視化用的地圖數據 (JSON 格式)

頻率限制: 0.8 Hz (與 scan_throttle 同步)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import time
import math
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TrustMapSolver(Node):
    def __init__(self):
        super().__init__('trust_map_solver')
        
        # 發佈器
        self.map_pub = self.create_publisher(String, '/viz_map_data', 10)
        self.manual_control_pub = self.create_publisher(String, '/manual_control', 10)  # 用於緊急停止
        
        # 修復 QoS 不兼容問題：使用 BEST_EFFORT 策略以兼容 topic_tools
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 訂閱 LiDAR 掃描數據
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_throttled',  # 訂閱已節流的掃描話題
            self.scan_callback,
            qos_profile
        )

        # 訂閱編碼器狀態（里程計）
        self.odom_sub = self.create_subscription(
            Twist,
            '/encoder_state',
            self.odom_callback,
            10
        )
        
        # TF 監聽器 (追蹤機器人位置)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 參數
        self.grid_size = 0.1        # 網格大小 (米)
        self.trust_max = 10         # 最大信任值 (約 4-5 秒達到)
        self.trust_threshold = 4    # 牆體確認閾值
        self.raycast_max_range = 5.0  # 最大計算距離
        
        # 時間衰減參數
        self.wall_confirm_time = 5.0     # 需要持續掃描 5 秒確認為牆體
        self.disappear_time = 1.5        # 1.5 秒未掃描到則消失
        self.trust_increase_rate = 2     # 每次掃描增加的信任值
        self.trust_decrease_rate = 0.5   # 每秒衰減的信任值
        
        # 機器人姿態
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.last_slam_pose = None
        self.last_slam_time = None
        self.last_odom_pose = None
        self.last_odom_time = None
        self.last_alert_time = 0.0
        self.slam_unstable = False  # 標記 SLAM 不穩定，暫停衰減
        self.slam_recovery_time = None
        
        # 頻率限制 (3.3 Hz)
        self.publish_interval = 0.3  # 秒 (0.3 秒更新一次)
        self.last_publish_time = time.time()
        self.scan_count = 0
        
        self.get_logger().info("Trust Map Solver Started - Processing YDLidar X2 data (3.3 Hz)")
        self.get_logger().info(f"Subscribing to: /scan_throttled")
        self.get_logger().info(f"Publishing to: /viz_map_data")
        
        # LiDAR 與車身之固定偏角（度數透過環境變數 LIDAR_YAW_DEG 設定，正為逆時針）
        try:
            yaw_deg = float(os.environ.get('LIDAR_YAW_DEG', '0'))
        except Exception:
            yaw_deg = 0.0
        self.lidar_yaw_offset_rad = yaw_deg * math.pi / 180.0
        # 記錄偏角（無論是否為 0，方便除錯）
        self.get_logger().info(f"LiDAR 偏角: {yaw_deg:.2f}° ({self.lidar_yaw_offset_rad:.4f} rad)")

    def scan_callback(self, msg):
        """
        處理 LiDAR 掃描數據的回調函數
        
        【時間衰減機制】
        - 牆體：需要重複掃描 4-6 秒才確認（黑色）
        - 空氣：未掃描到 1-2 秒則消失
        - 灰色區域：超出掃描範圍保持最後狀態
        
        【全局座標轉換】
        - 透過 TF 獲取機器人在 map frame 的位置
        - 將掃描點從 laser frame 轉換為 map frame
        - 支援機器人移動時的地圖建立
        
        原理:
        1. 獲取機器人當前位置 (map -> base_link)
        2. 遍歷每一條掃描射線
        3. 轉換為全局座標
        4. 命中點增加信任值 + 更新時間戳
        5. 空氣點設為 0（白色）+ 更新時間戳
        6. 定期清理過期數據
        """
        self.scan_count += 1
        hit_points = []
        free_points = []
        current_time = time.time()
        
        # 【關鍵】獲取機器人當前位置（全局座標）
        try:
            # 嘗試從 TF 樹獲取 map -> base_link 的變換
            transform = self.tf_buffer.lookup_transform(
                'map',          # 目標座標系
                'base_link',    # 源座標系
                rclpy.time.Time()  # 最新時間
            )
            
            # 更新機器人全局位置
            self.robot_pose['x'] = transform.transform.translation.x
            self.robot_pose['y'] = transform.transform.translation.y
            
            # 從四元數計算航向角
            quat = transform.transform.rotation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            self.robot_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException as ex:
            # TF 不可用時，使用相對座標（原地建圖模式）
            if self.scan_count % 50 == 1:  # 每 50 次掃描提示一次
                self.get_logger().warn(f'無法獲取 TF: {ex}，使用相對座標模式')
        
        robot_x = self.robot_pose['x']
        robot_y = self.robot_pose['y']
        robot_theta = self.robot_pose['theta']

        # --- A. 座標跳變偵測 (Anti-Jump Monitor) ---
        now_time = time.time()
        if self.last_slam_pose is not None and self.last_slam_time is not None:
            dt = now_time - self.last_slam_time
            if dt >= 0.5:  # 半秒以上才檢查，避免過於頻繁
                dx = robot_x - self.last_slam_pose['x']
                dy = robot_y - self.last_slam_pose['y']
                dist = math.hypot(dx, dy)
                if dist > 0.2:  # 超過 20 公分視為跳變
                    # 【軟化反應】：標記 SLAM 不穩定，暫停衰減而非立即停止
                    self.get_logger().warn(f"Anti-Jump: SLAM 跳變 {dist:.2f} m，暫停衰減等待恢復")
                    self.slam_unstable = True
                    self.slam_recovery_time = now_time
                else:
                    # 如果穩定 0.3 秒以上，取消不穩定標記
                    if self.slam_unstable and (now_time - self.slam_recovery_time) > 0.3:
                        self.get_logger().info("SLAM 已恢復穩定")
                        self.slam_unstable = False
        self.last_slam_pose = {'x': robot_x, 'y': robot_y, 'theta': robot_theta}
        self.last_slam_time = now_time
        
        # 處理每一條掃描射線
        for i, distance in enumerate(msg.ranges):
            # 過濾無效數據
            if distance == float('inf') or distance == 0 or math.isnan(distance):
                continue
            
            # 計算射線角度（相對於機器人）
            angle = msg.angle_min + i * msg.angle_increment + self.lidar_yaw_offset_rad
            
            # 限制最大計算距離（優化性能）
            calc_distance = min(distance, self.raycast_max_range)
            
            # 轉換為機器人局部座標
            local_x = calc_distance * math.cos(angle)
            local_y = calc_distance * math.sin(angle)
            
            # 【關鍵】轉換為全局座標（考慮機器人位置和朝向）
            global_x = robot_x + local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta)
            global_y = robot_y + local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta)
            
            # 記錄命中點（全局座標）
            hit_points.append((global_x, global_y))
            
            # 射線投射：從機器人當前位置到命中點之間的格子為空氣
            # 使用簡單的 Bresenham-like 算法
            num_steps = max(int(calc_distance / self.grid_size), 1)
            for step in range(1, num_steps):
                t = step / num_steps
                # 插值局部座標
                interp_local_x = t * local_x
                interp_local_y = t * local_y
                # 轉換為全局座標
                free_global_x = robot_x + interp_local_x * math.cos(robot_theta) - interp_local_y * math.sin(robot_theta)
                free_global_y = robot_y + interp_local_x * math.sin(robot_theta) + interp_local_y * math.cos(robot_theta)
                free_points.append((free_global_x, free_global_y))
        
        # 只發佈當前掃描結果，不累積歷史
        if current_time - self.last_publish_time >= self.publish_interval:
            cell_map = {}
            for px, py in hit_points:
                key = (int(round(px / self.grid_size)), int(round(py / self.grid_size)))
                cell_map[key] = {'x': key[0], 'y': key[1], 's': self.trust_threshold}
            for px, py in free_points:
                key = (int(round(px / self.grid_size)), int(round(py / self.grid_size)))
                if key in cell_map:
                    continue  # 命中優先
                cell_map[key] = {'x': key[0], 'y': key[1], 's': 0}

            self.publish_map(list(cell_map.values()))
            self.last_publish_time = current_time
            self.get_logger().info(f"Map published (scan #{self.scan_count}, cells: {len(cell_map)})")

    def publish_map(self, map_cells):
        """發佈當前掃描結果（不累積歷史）。"""
        payload = {
            'robot': self.robot_pose,
            'map': map_cells
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.map_pub.publish(msg)

    # =========================
    # 里程計訂閱與一致性檢查
    # =========================
    def odom_callback(self, msg: Twist):
        now_time = time.time()
        odom_pose = {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'theta': msg.angular.z
        }

        if self.last_odom_pose is not None and self.last_odom_time is not None:
            dt = max(1e-6, now_time - self.last_odom_time)
            odom_dx = odom_pose['x'] - self.last_odom_pose['x']
            odom_dy = odom_pose['y'] - self.last_odom_pose['y']
            odom_dist = math.hypot(odom_dx, odom_dy)

            slam_dist = 0.0
            if self.last_slam_pose is not None:
                slam_dx = self.robot_pose['x'] - self.last_slam_pose['x']
                slam_dy = self.robot_pose['y'] - self.last_slam_pose['y']
                slam_dist = math.hypot(slam_dx, slam_dy)

            # --- B. 里程計與 SLAM 差值比對 ---
            if odom_dist >= 0.10 and slam_dist <= 0.02:
                # 打滑或硬體異常
                if now_time - self.last_alert_time > 2.0:  # 避免刷屏
                    self.last_alert_time = now_time
                    self.get_logger().error(
                        f"ConsistencyCheck: Odom移動 {odom_dist:.2f} m，但 SLAM {slam_dist:.2f} m，判定打滑/硬體異常，任務中斷"
                    )
                    self.publish_stop(reason="odom_slam_mismatch")

        self.last_odom_pose = odom_pose
        self.last_odom_time = now_time

    # =========================
    # 共用：發送停止指令
    # =========================
    def publish_stop(self, reason: str = ""):
        msg = String()
        msg.data = 'STOP'
        self.manual_control_pub.publish(msg)
        self.get_logger().warn(f"發布 STOP 指令 (reason={reason})")


def main(args=None):
    rclpy.init(args=args)
    node = TrustMapSolver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
