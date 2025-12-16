import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, Motor

class PiMotorDriver(Node):
    def __init__(self):
        super().__init__('pi_motor_driver')
        
        # --- 參數 ---
        self.WHEEL_SEPARATION = 0.15 
        self.MAX_PWM = 1.0  # 提升至最大 PWM (100% 動力)
        # 根據測試結果：PWM=0.7 時線速度 = 0.375 m/s (60cm/1.6s)
        self.V_MAX_REFERENCE = 0.375 / 0.7  # 約 0.535 m/s @ PWM=1.0
        
        # --- GPIO 初始化 (與前一版相同) ---
        self.motor_l_pwm = PWMOutputDevice(18)
        self.motor_l_dir = Motor(forward=14, backward=15)
        self.motor_r_pwm = PWMOutputDevice(22)
        self.motor_r_dir = Motor(forward=17, backward=27)

        # --- ROS 訂閱 ---
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info('馬達驅動節點啟動，訂閱 /cmd_vel')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 運動學解算：線速度和角速度 -> 左右輪線速度
        V_left = linear_x - (angular_z * self.WHEEL_SEPARATION / 2.0)
        V_right = linear_x + (angular_z * self.WHEEL_SEPARATION / 2.0)
        
        # 線性映射 V -> PWM (0.0 to 1.0)
        pwm_left = min(abs(V_left / self.V_MAX_REFERENCE), self.MAX_PWM)
        pwm_right = min(abs(V_right / self.V_MAX_REFERENCE), self.MAX_PWM)
        
        # 輸出到 GPIO
        self._set_wheel_speed(self.motor_l_dir, self.motor_l_pwm, V_left, pwm_left)
        self._set_wheel_speed(self.motor_r_dir, self.motor_r_pwm, V_right, pwm_right)

    def _set_wheel_speed(self, motor_dir, motor_pwm, V_wheel, pwm_val):
        """通用設置單輪速度"""
        if V_wheel > 0:
            motor_dir.forward()
        elif V_wheel < 0:
            motor_dir.backward()
        else:
            motor_dir.stop()
        motor_pwm.value = pwm_val

    def stop_motors(self):
        self._set_wheel_speed(self.motor_l_dir, self.motor_l_pwm, 0, 0)
        self._set_wheel_speed(self.motor_r_dir, self.motor_r_pwm, 0, 0)


def main(args=None):
    rclpy.init(args=args)
    node = PiMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()