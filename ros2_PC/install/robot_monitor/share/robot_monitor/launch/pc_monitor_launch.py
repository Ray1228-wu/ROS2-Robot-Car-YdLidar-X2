import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 定義話題名稱
LIDAR_TOPIC = '/scan'
THROTTLED_TOPIC = '/scan_throttled'

def generate_launch_description():
    
    # 1. 啟動 Rosbridge (WebSocket Port 9090)
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen'
    )

    # 2. 啟動 Throttle 節點 (關鍵步驟)
    # 將 Lidar 的 /scan 數據，以 1.0 Hz 的頻率，發布到 /scan_throttled
    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttle',
        output='screen',
        # arguments 對應指令: ros2 run topic_tools throttle messages /scan 1.0 /scan_throttled
        arguments=['messages', '/scan', '0.8', '/scan_throttled']
        )

    # 3. 啟動 SLAM Toolbox (訂閱低頻的節流話題)
    slam_toolbox = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[
            {'use_sim_time': False},
            # ... (保留原有的 frame 設定) ...
            {'scan_topic': THROTTLED_TOPIC},
            {'mode': 'mapping'},
            
            # --- 強制更新頻率 (保留之前的設定) ---
            {'map_update_interval': 0.5},
            {'minimum_travel_distance': 0.0},
            
            # ▼▼▼ 解決問題 1：快速清除動態障礙物 ▼▼▼
            # 原理：當雷射穿過一個原本是黑點的地方，扣分的力道要加大
            {'map_file_name': 'my_map'},
            {'map_start_pose': [0.0, 0.0, 0.0]},
            
            # 關鍵參數：
            # 命中機率 (0.5 ~ 1.0): 偵測到障礙物時，信心增加多少 (建議 0.65)
            {'probability_hit': 0.65}, 
            
            # 失誤機率 (0.0 ~ 0.5): 雷射穿過去時，信心減少多少 
            # 預設通常是 0.45，改小一點 (例如 0.4) 會讓它更容易被清除
            # 或是維持 0.45 但配合下面的 clamp
            {'probability_miss': 0.4}, 
            
            # 信心值上下限：讓狀態更容易在「有」跟「沒有」之間切換
            {'clamp_min': 0.12}, 
            {'clamp_max': 0.97},
            # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
        ]
)

    # 4. 發布靜態座標轉換
    fake_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    tf2_web_republisher = Node(
        package='tf2_web_republisher',
        executable='tf2_web_republisher_node',
        name='tf2_web_republisher'
    )

    return LaunchDescription([
        rosbridge,
        fake_odom_tf,
        throttle_node, 
        slam_toolbox,
        tf2_web_republisher
    ])