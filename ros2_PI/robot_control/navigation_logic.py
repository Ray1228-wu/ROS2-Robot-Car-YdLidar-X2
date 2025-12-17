import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    import tf_transformations
except ImportError:
    raise SystemExit("Missing dependency tf_transformations. Install with: sudo apt install ros-humble-tf-transformations (preferred) or pip install tf-transformations")
import math

class PiNavigationLogic(Node):
    def __init__(self):
        super().__init__('pi_navigation_logic')

        # --- 導航參數 ---
        self.FORWARD_SPEED = 0.25  # m/s (線速度) - 根據測試結果調整
        self.TURN_SPEED = 0.5       # rad/s (角速度)
        self.TURN_THRESHOLD = math.radians(17) # 17度轉向閾值
        self.GOAL_TOLERANCE = 0.1   # 到達目標容忍度 (0.1m)

        # --- 狀態 ---
        self.state = "IDLE" 
        self.target_x, self.target_y = 0.0, 0.0
        self.current_x, self.current_y, self.current_theta = 0.0, 0.0, 0.0

        # --- 訂閱 ---
        self.sub_goal = self.create_subscription(PoseStamped, '/nav_goal', self.goal_callback, 10)
        self.sub_control = self.create_subscription(String, '/motor_control', self.control_callback, 10)
        # 訂閱來自 pi_encoder_reader 的標準 Odometry 訊息
        self.sub_odom = self.create_subscription(Odometry, '/odometry/local', self.odom_callback, 10)

        # --- 發佈 ---
        # 與馬達驅動對齊：馬達訂閱 /cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/robot_status', 10)

        # --- 定時器 ---
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz 控制循環
        self.status_timer = self.create_timer(0.5, self.status_loop)    # 2 Hz 狀態報告

        self.get_logger().info("導航邏輯節點啟動")

    # (goal_callback, control_callback, status_loop 函數與單一節點版類似)

    def odom_callback(self, msg):
        """接收當前位置信息"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # 將四元數轉為歐拉角 (yaw)
        _, _, self.current_theta = tf_transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
    
    def control_loop(self):
        if self.state != "NAVIGATING":
            # 停止馬達
            self.pub_cmd_vel.publish(Twist())
            return
            
        # --- 導航邏輯 ---
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.GOAL_TOLERANCE:
            self.state = "REACHED"
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("到達目標！")
            return
            
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) # 正規化

        twist = Twist()
        if abs(angle_error) > self.TURN_THRESHOLD:
            # 轉向模式 (角速度優先)
            twist.angular.z = math.copysign(self.TURN_SPEED, angle_error)
            twist.linear.x = 0.0
        else:
            # 前進模式 (帶有角度修正)
            twist.linear.x = self.FORWARD_SPEED * (distance / 1.0) # 距離越遠走越快，最多 V_forward
            twist.linear.x = min(twist.linear.x, self.FORWARD_SPEED)
            twist.angular.z = angle_error * 1.5 # P 增益 1.5
            
        self.pub_cmd_vel.publish(twist)
    
    def goal_callback(self, msg):
        """接收導航目標"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.state = "NAVIGATING"
        self.get_logger().info(f"收到新目標: ({self.target_x:.2f}, {self.target_y:.2f})")
    
    def control_callback(self, msg):
        """接收控制命令 (暫停/繼續/取消)"""
        cmd = msg.data.lower()
        if cmd == "pause":
            self.state = "PAUSED"
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("導航暫停")
        elif cmd == "resume":
            self.state = "NAVIGATING"
            self.get_logger().info("導航繼續")
        elif cmd == "cancel":
            self.state = "IDLE"
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("導航取消")
    
    def status_loop(self):
        """定期發布機器人狀態"""
        status_msg = String()
        status_msg.data = f"{self.state}|{self.current_x:.2f}|{self.current_y:.2f}|{self.current_theta:.2f}"
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PiNavigationLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("導航節點中斷，停止馬達")
    except Exception as e:
        node.get_logger().error(f"導航節點異常: {e}，停止馬達")
    finally:
        # 確保馬達停止
        stop_cmd = Twist()
        node.pub_cmd_vel.publish(stop_cmd)
        node.get_logger().info("已發送馬達停止指令")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()