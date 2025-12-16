import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry # 標準 Odometry 訊息
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from gpiozero import DigitalInputDevice
import math
import time
import tf_transformations

class PiEncoderReader(Node):
    def __init__(self):
        super().__init__('pi_encoder_reader')
        
        # --- 參數 ---
        self.WHEEL_RADIUS = 0.0325  # 輪子半徑 (m) - 6.5cm 輪直徑
        self.WHEEL_SEPARATION = 0.15 
        self.TICKS_PER_REV = 400    # 每圈脈衝數
        
        # --- 狀態 ---
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.tick_l, self.tick_r = 0, 0
        self.last_time = self.get_clock().now()
        
        # --- GPIO 初始化 ---
        self.enc_l_a = DigitalInputDevice(2, pull_up=True)
        self.enc_l_b = DigitalInputDevice(3, pull_up=True)
        self.enc_r_a = DigitalInputDevice(23, pull_up=True)
        self.enc_r_b = DigitalInputDevice(24, pull_up=True)

        self.enc_l_a.when_activated = self._on_left_encode
        self.enc_r_a.when_activated = self._on_right_encode

        # --- ROS 發佈 ---
        # 1. 發佈給 Web 端 (Twist 格式)
        self.pub_web = self.create_publisher(Twist, '/encoder_state', 10)
        # 2. 發佈標準 Odometry (nav_msgs/Odometry) 給 ROS 系統用
        self.pub_odom = self.create_publisher(Odometry, '/odometry/local', 10)
        
        self.timer = self.create_timer(0.1, self.publish_odometry) # 10 Hz
        self.get_logger().info('編碼器讀取節點啟動')

    def _on_left_encode(self):
        """左輪編碼器中斷 - 往前時遞減"""
        # 根據測試結果：往前時兩輪遞減
        self.tick_l -= 1

    def _on_right_encode(self):
        """右輪編碼器中斷 - 往前時遞減"""
        # 根據測試結果：往前時兩輪遞減
        self.tick_r -= 1

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # 計算每輪行進的距離
        dist_l = (self.tick_l / self.TICKS_PER_REV) * 2 * math.pi * self.WHEEL_RADIUS
        dist_r = (self.tick_r / self.TICKS_PER_REV) * 2 * math.pi * self.WHEEL_RADIUS
        
        # 中心行進距離和轉向角度
        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / self.WHEEL_SEPARATION
        
        # 更新位置和方向
        if abs(d_theta) > 1e-6:
            # 弧形運動
            radius = d_center / d_theta if abs(d_theta) > 1e-9 else float('inf')
            self.x += radius * math.sin(self.theta + d_theta) - radius * math.sin(self.theta)
            self.y += -radius * math.cos(self.theta + d_theta) + radius * math.cos(self.theta)
        else:
            # 直線運動
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # 正規化到 [-pi, pi]
        
        # 重置刻度計數
        self.tick_l = 0
        self.tick_r = 0
        self.last_time = current_time
        
        # --- 1. 發佈給 Web 端 (Twist 格式) ---
        web_msg = Twist()
        web_msg.linear.x = self.x
        web_msg.linear.y = self.y
        web_msg.angular.z = self.theta
        self.pub_web.publish(web_msg)
        
        # --- 2. 發佈標準 Odometry (可供其他 ROS 導航模組使用) ---
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 將 theta (yaw) 轉為四元數
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # 速度估計
        odom.twist.twist.linear.x = d_center / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = d_theta / dt if dt > 0 else 0.0
        
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PiEncoderReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()