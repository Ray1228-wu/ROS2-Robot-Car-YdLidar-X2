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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import json
import time
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TrustMapSolver(Node):
    def __init__(self):
        super().__init__('trust_map_solver')
        
        # 發佈器
        self.map_pub = self.create_publisher(String, '/viz_map_data', 10)
        
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
        
        # 地圖資料
        self.grid_map = {}          # 格式: {(x,y): {'trust': float, 'last_seen': time}}
        self.current_scan_cells = set()  # 當前掃描範圍內的格子
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # 頻率限制 (1.25 Hz，與 scan_throttle 同步)
        self.publish_interval = 0.8  # 秒 (1.25 Hz)
        self.last_publish_time = time.time()
        self.scan_count = 0
        
        self.get_logger().info("Trust Map Solver Started - Processing YDLidar X2 data (1.25 Hz)")
        self.get_logger().info(f"Subscribing to: /scan_throttled")
        self.get_logger().info(f"Publishing to: /viz_map_data")

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
        self.current_scan_cells = set()  # 重置當前掃描範圍
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
        
        # 處理每一條掃描射線
        for i, distance in enumerate(msg.ranges):
            # 過濾無效數據
            if distance == float('inf') or distance == 0 or math.isnan(distance):
                continue
            
            # 計算射線角度（相對於機器人）
            angle = msg.angle_min + i * msg.angle_increment
            
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
        
        # 更新地圖
        self.update_map(hit_points, free_points, current_time)
        
        # 清理過期數據（未掃描到超過 disappear_time 的格子）
        self.cleanup_old_cells(current_time)
        
        # 按照時間限制發佈
        if current_time - self.last_publish_time >= self.publish_interval:
            self.publish_map()
            self.last_publish_time = current_time
            active_cells = len([k for k, v in self.grid_map.items() if v['trust'] > 0])
            self.get_logger().info(f"Map published (scan #{self.scan_count}, active cells: {active_cells})")

    def update_map(self, hit_points, free_points, current_time):
        """
        【時間衰減版】更新網格地圖的信心度
        
        核心邏輯：
        1. 牆體（命中點）→ 增加信任值 + 更新時間戳
        2. 空氣（自由空間）→ 設為 0 + 更新時間戳
        3. 未掃描到的格子 → 由 cleanup_old_cells 處理
        """
        # 1. 處理牆壁 (命中點)
        for px, py in hit_points:
            key = (int(round(px / self.grid_size)), int(round(py / self.grid_size)))
            
            if key not in self.grid_map:
                self.grid_map[key] = {'trust': 0, 'last_seen': current_time}
            
            # 增加信任值（重複掃描累積）
            old_trust = self.grid_map[key]['trust']
            new_trust = min(old_trust + self.trust_increase_rate, self.trust_max)
            self.grid_map[key] = {'trust': new_trust, 'last_seen': current_time}
            
            # 記錄當前掃描範圍
            self.current_scan_cells.add(key)

        # 2. 處理空氣 (自由空間)
        for px, py in free_points:
            key = (int(round(px / self.grid_size)), int(round(py / self.grid_size)))
            
            # 記錄當前掃描範圍
            self.current_scan_cells.add(key)
            
            # 空氣區域設為 0（白色）
            self.grid_map[key] = {'trust': 0, 'last_seen': current_time}
    
    def cleanup_old_cells(self, current_time):
        """
        清理過期數據
        
        規則：
        - 未掃描到超過 disappear_time (1-2秒) 的格子 → 移除
        - 確保地圖保持動態更新
        """
        keys_to_remove = []
        
        for key, data in self.grid_map.items():
            time_since_seen = current_time - data['last_seen']
            
            # 超過消失時間且不在當前掃描範圍 → 移除
            if time_since_seen > self.disappear_time and key not in self.current_scan_cells:
                keys_to_remove.append(key)
        
        # 刪除過期格子
        for key in keys_to_remove:
            del self.grid_map[key]

    def publish_map(self):
        """
        【時間衰減版】發佈地圖數據到網頁
        
        輸出格式:
        {
            "robot": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "map": [
                {"x": -5, "y": -8, "s": 4},  // s: score (信心度)
                {"x": -5, "y": -7, "s": 0},
                ...
            ]
        }
        
        顏色映射：
        - s >= 4: 黑色（確認牆體，需要 4-5 秒掃描）
        - s = 0: 白色（空氣）
        - 其他: 灰色（未知/偵測範圍外）
        """
        # 打包資料
        output_list = []
        
        for key, data in self.grid_map.items():
            trust = data['trust']
            
            # 牆體 (信心度 >= 閾值) - 黑色
            if trust >= self.trust_threshold:
                output_list.append({'x': key[0], 'y': key[1], 's': int(trust)})
            # 空氣 (信心度 = 0) - 白色
            elif trust == 0:
                output_list.append({'x': key[0], 'y': key[1], 's': 0})
        
        # 組合 JSON 消息
        payload = {
            'robot': self.robot_pose,
            'map': output_list
        }
        
        msg = String()
        msg.data = json.dumps(payload)
        self.map_pub.publish(msg)


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
