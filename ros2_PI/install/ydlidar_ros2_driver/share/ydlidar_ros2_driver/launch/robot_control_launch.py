#!/usr/bin/python3
"""
ROS2 Launch 檔案 - 啟動 YDLidar + 靜態 TF
使用方式: ros2 launch ydlidar_ros2_driver robot_control_launch.py
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """生成 launch description"""

    # 取得 ydlidar 套件路徑
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')

    # 宣告 launch 參數
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # YDLidar 掃描驅動節點 (Lifecycle)
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        # 與參數檔節點名稱一致，確保 baudrate/port 等參數生效
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )
    
    # TF2 靜態變換發佈器 (連接 base_link 和 laser_frame)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen',
    )
    
    
    
    
    
    # 啟動信息
    launch_info = LogInfo(
        msg="🚀 啟動 YDLidar + 靜態 TF"
    )
    
    return LaunchDescription([
        launch_info,
        params_declare,
        driver_node,
        static_tf_node,
    ])
