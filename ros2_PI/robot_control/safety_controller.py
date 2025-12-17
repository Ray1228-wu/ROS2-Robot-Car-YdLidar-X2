import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        # --- 參數設定 ---
        self.WARNING_DIST = 0.35  # 35cm 開始減速 (降低干預距離)
        self.DANGER_DIST = 0.20   # 20cm 強制避障 (停車或轉向)
        self.CHECK_ANGLE = math.radians(25) # 檢測角度 +/- 25度
        
        # --- 狀態 ---
        self.nav_cmd = Twist() # 儲存導航節點原本想做的動作
        self.last_nav_time = self.get_clock().now()

        # --- 訂閱 ---
        # 1. 訂閱雷達 (建議用 throttled 版本以節省效能)
        self.sub_scan = self.create_subscription(
            LaserScan, 
            '/scan_throttled', 
            self.scan_callback, 
            10)
            
        # 2. 訂閱導航節點的 "建議" 速度
        # 注意：因為 navigation_logic 現在直接發布到 /cmd_vel，
        # 如果要使用 safety_controller，需要讓 navigation_logic 改發布到 /cmd_vel_nav
        self.sub_nav_cmd = self.create_subscription(
            Twist, 
            '/cmd_vel_nav', 
            self.nav_cmd_callback, 
            10)

        # --- 發佈 ---
        # 3. 發佈 "最終" 決定給馬達的指令
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("安全控制器已啟動 (優先權: 避障 > 導航)")

    def nav_cmd_callback(self, msg):
        """接收導航節點的指令，暫存起來，不直接發送"""
        self.nav_cmd = msg
        self.last_nav_time = self.get_clock().now()

    def scan_callback(self, msg):
        """核心避障邏輯"""
        # 1. 解析雷達數據，找出前方區域的最小距離
        min_dist = float('inf')
        obstacle_angle = 0.0
        
        # 計算索引範圍 (假設雷達是逆時針掃描，0度在正前方或正後方，需根據您的雷達調整)
        # 這裡假設 RPLidar: 0度是正前方
        # 我們需要檢查 [0 ~ 25度] 和 [335 ~ 360度]
        
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        
        # 將 +/- 25度轉換為 array index
        # 注意：這取決於您的雷達 msg.angle_min 是 0 還是 -pi
        # 通用的做法是遍歷並過濾角度
        
        left_dist = float('inf')  # 左側空曠度
        right_dist = float('inf') # 右側空曠度

        for i, dist in enumerate(ranges):
            if dist == float('inf') or dist == 0:
                continue
                
            # 計算該點的角度 (正規化到 -pi ~ pi)
            angle = msg.angle_min + i * angle_increment
            
            # 確保角度在 -pi 到 pi 之間
            while angle > math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi
            
            # 檢查是否在前方 +/- 25度範圍內
            if -self.CHECK_ANGLE < angle < self.CHECK_ANGLE:
                if dist < min_dist:
                    min_dist = dist
                    obstacle_angle = angle
            
            # 順便偵測左右兩側稍微寬一點的區域 (用於決定轉向方向)
            # 左側: 25 ~ 90 度
            if self.CHECK_ANGLE < angle < math.pi/2:
                if dist < left_dist: left_dist = dist
            # 右側: -90 ~ -25 度
            if -math.pi/2 < angle < -self.CHECK_ANGLE:
                if dist < right_dist: right_dist = dist

        # 2. 決策樹 (Decision Tree)
        final_cmd = Twist()
        
        # 檢查導航指令是否超時 (如果導航掛了，就停車)
        if (self.get_clock().now() - self.last_nav_time).nanoseconds > 1e9: # 1秒超時
            self.pub_cmd_vel.publish(Twist()) # 停車
            return

        if min_dist < self.DANGER_DIST:
            # === 危險模式 (Distance < 30cm) ===
            # 強制接管！無視導航指令
            self.get_logger().warn(f"避障觸發！前方障礙 {min_dist:.2f}m")
            
            final_cmd.linear.x = 0.0 # 絕對不能前進
            final_cmd.angular.z = 0.0
            
            # 決定轉向：往比較空曠的那邊轉
            # 為了避免在死胡同抖動，可以加一個遲滯或旋轉固定時間，這裡先做簡單判斷
            
            # 判斷邏輯：
            # 如果原本導航就想轉彎，且那個方向是安全的，就加速轉過去
            # 如果原本想直走，就往空曠處轉
            
            if left_dist > right_dist:
                # 左邊比較空，往左轉
                final_cmd.angular.z = 0.5 
            else:
                # 右邊比較空，往右轉
                final_cmd.angular.z = -0.5
                
        elif min_dist < self.WARNING_DIST:
            # === 警告模式 (30cm < Distance < 60cm) ===
            # 減速放行
            # self.get_logger().info(f"減速中... 前方 {min_dist:.2f}m")
            
            # 速度打折 (距離越近越慢)
            # 簡單線性比例: (current - 0.3) / (0.6 - 0.3) -> 0.0 ~ 1.0
            scale_factor = (min_dist - self.DANGER_DIST) / (self.WARNING_DIST - self.DANGER_DIST)
            scale_factor = max(0.1, scale_factor) # 最低保留 10% 速度
            
            final_cmd.linear.x = self.nav_cmd.linear.x * scale_factor
            final_cmd.angular.z = self.nav_cmd.angular.z # 轉向不減速，以此保持閃避能力
            
        else:
            # === 安全模式 ===
            # 直接放行導航指令
            final_cmd = self.nav_cmd

        # 3. 發布最終指令
        self.pub_cmd_vel.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("安全控制器中斷，停止馬達")
    except Exception as e:
        node.get_logger().error(f"安全控制器異常: {e}，停止馬達")
    finally:
        # 確保馬達停止
        stop_cmd = Twist()
        node.pub_cmd_vel.publish(stop_cmd)
        node.get_logger().info("已發送馬達停止指令")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()