#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Monitor - 統一啟動系統
管理所有必要服務：rosbridge, throttle, pc_monitor, trust_map_solver, HTTP server
"""


import subprocess
import time
import os
import signal
import sys
import socket
import math

def check_port(port, host='127.0.0.1'):
    """檢查埠是否開放"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except:
        return False

def run_command(cmd, description, show_output=False):
    """執行命令並返回進程"""
    print(f"\n{description}...")
    try:
        # 準備環境：確保 ROS_DOMAIN_ID 傳遞到子進程
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '30'  # ← 改為 30 與 PI 通讯
        
        process = subprocess.Popen(
            cmd,
            shell=True,
            preexec_fn=os.setsid,
            env=env,  # ← 傳遞完整的環境變數
            stdout=subprocess.PIPE if show_output else subprocess.DEVNULL,
            stderr=subprocess.PIPE if show_output else subprocess.DEVNULL,
            text=True
        )
        print(f"✅ {description} 已啟動 (PID: {process.pid})")
        return process
    except Exception as e:
        print(f"❌ {description} 失敗: {e}")
        return None

def wait_for_service(port, service_name, timeout=20):
    """等待服務埠開放"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        if check_port(port):
            print(f"✅ {service_name} 已就緒 (埠 {port})")
            return True
        time.sleep(0.2)
    print(f"⚠️  {service_name} 未在時間內開放 (埠 {port})")
    return False

def setup_ros_env():
    """設定 ROS 2 環境變數"""
    os.environ['ROS_DOMAIN_ID'] = '30'  # ← 與 PI 同步 (DOMAIN_ID=30)
    os.environ['ROS_LOCALHOST_ONLY'] = '0'  # 允許跨機器通信
    # 預設 LiDAR 與車頭偏角：往右 120 度（順時針，故 -120）
    os.environ.setdefault('LIDAR_YAW_DEG', '-120')
    print(f"🧭 LIDAR_YAW_DEG = {os.environ['LIDAR_YAW_DEG']}° (可透過環境變數覆寫)")
    os.makedirs('/tmp/ros_logs', exist_ok=True)

def get_workspace_root():
    """取得工作區根目錄（相對於本腳本位置）"""
    # 取得 start_all.py 所在的目錄
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 直接使用腳本所在目錄作為工作區根目錄，避免跳到父層導致路徑錯誤
    return script_dir

def main():
    print("=" * 70)
    print("🤖 Robot Monitor - 統一啟動系統")
    print("=" * 70)
    
    # 獲取工作區路徑
    ws_root = get_workspace_root()
    print(f"\n📁 工作區: {ws_root}\n")
    
    # 設置環境
    setup_ros_env()
    
    # 清理舊進程（包含殘留 throttle/rosbridge）
    print("\n🛑 停止舊進程...")
    os.system("pkill -9 -f 'topic_tools/throttle' 2>/dev/null || true")
    os.system("pkill -9 -f 'scan_throttle' 2>/dev/null || true")
    os.system("pkill -9 -f 'ros2 run.*throttle' 2>/dev/null || true")
    os.system("pkill -9 -f 'ros2 launch' 2>/dev/null || true")
    os.system("pkill -9 -f 'python3.*trust_map' 2>/dev/null || true")
    os.system("pkill -9 -f 'python3.*http.server' 2>/dev/null || true")
    os.system("pkill -9 -f rosbridge 2>/dev/null || true")
    time.sleep(2)
    print("✅ 舊進程已停止")
    
    # 清理 ROS 環境
    print("\n🧹 清潔 ROS 2 環境...")
    os.system("rm -rf ~/.ros/ /tmp/ros_logs/* 2>/dev/null || true")
    print("✅ ROS 環境已清潔")
    
    processes = []
    
    # === 1. rosbridge_websocket (埠 9090) ===
    print("\n【啟動服務 1/5】")
    proc = run_command(
        "bash -c 'source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml'",
        "🌐 rosbridge_websocket"
    )
    if proc:
        processes.append(proc)
        wait_for_service(9090, "WebSocket", timeout=20)
        time.sleep(2)
    
    # === 2. throttle 節點 (獨立管理 - 關鍵修正) ===
    print("\n【啟動服務 2/5】")
    proc = run_command(
        f"bash -c 'source /opt/ros/humble/setup.bash && cd {ws_root} && source install/setup.bash && ROS_DOMAIN_ID=30 ros2 run topic_tools throttle messages /scan 0.8 /scan_throttled'",
        "📊 throttle (/scan → /scan_throttled @ 0.8 Hz)",
        show_output=True  # ← 顯示輸出以便診斷
    )
    if proc:
        processes.append(proc)
    time.sleep(3)  # ← 增加等待時間確保 throttle 初始化
    
    # === 3. pc_monitor_launch ===
    print("\n【啟動服務 3/5】")
    # === 3. static TF: base_link -> laser（以環境變數設定偏角，單位度） ===
    try:
        yaw_deg = float(os.environ.get('LIDAR_YAW_DEG', '0'))
    except Exception:
        yaw_deg = 0.0
    yaw_rad = yaw_deg * 3.141592653589793 / 180.0
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    base_frame = os.environ.get('BASE_FRAME', 'base_link')
    lidar_frame = os.environ.get('LIDAR_FRAME', 'laser')
    print(f"   ↪ static TF: {base_frame} → {lidar_frame} (yaw={yaw_deg:.2f}° / {yaw_rad:.5f} rad)")
    proc = run_command(
        "bash -c 'source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 "
        + f"{qz:.6f} {qw:.6f} " + f"{base_frame} {lidar_frame}'",
        "🔗 static_transform_publisher (base_link→laser)"
    )
    if proc:
        processes.append(proc)
    time.sleep(1)
    proc = run_command(
        f"bash -c 'source /opt/ros/humble/setup.bash && cd {ws_root} && source install/setup.bash && ros2 launch robot_monitor pc_monitor_launch.py'",
        "🎯 pc_monitor_launch (SLAM + TF)"
    )
    if proc:
        processes.append(proc)
    time.sleep(5)
    
    # === 4. trust_map_solver ===
    print("\n【啟動服務 4/5】")
    proc = run_command(
        f"bash -c 'source /opt/ros/humble/setup.bash && cd {ws_root} && source install/setup.bash && python3 robot_monitor/launch/trust_map_solver.py'",
        "🗺️  trust_map_solver (/viz_map_data @ 1.25 Hz)"
    )
    if proc:
        processes.append(proc)
    time.sleep(2)
    
    # === 5. HTTP 伺服器 (埠 8000) ===
    print("\n【啟動服務 5/5】")
    proc = run_command(
        f"bash -c 'cd {ws_root}/robot_monitor/web && python3 -m http.server 8000'",
        "🌍 HTTP 伺服器"
    )
    if proc:
        processes.append(proc)
        # HTTP server 通常很快啟動，但不需要 blocking wait
        time.sleep(0.5)  # 給它 0.5 秒初始化
    time.sleep(1)
    
    # === 驗證 ===
    print("\n" + "=" * 70)
    print("✅ 所有服務已啟動")
    print("=" * 70)
    
    print("\n【狀態檢查】")
    if check_port(9090):
        print("  ✅ WebSocket (9090): 已開放")
    else:
        print("  ❌ WebSocket (9090): 未開放")
    
    if check_port(8000):
        print("  ✅ HTTP (8000): 已開放")
    else:
        print("  ❌ HTTP (8000): 未開放")
    
    print("\n【進程驗證】")
    os.system("ps aux | grep -E 'rosbridge|topic_tools/throttle|trust_map_solver|http.server|pc_monitor_launch|static_transform_publisher' | grep -v grep | wc -l | xargs echo '  運行進程數:'")
    
    # === 訪問信息 ===
    print("\n【🚀 系統已啟動】")
    print("=" * 70)
    print("\n📍 訪問地址:")
    print("  🌐 http://localhost:8000/")
    print("\n📋 運行的服務:")
    print("  ✅ rosbridge_websocket   (埠 9090)")
    print("  ✅ throttle node          (/scan → /scan_throttled @ 0.8 Hz)")
    print("  ✅ static TF               (base_link → laser)")
    print("  ✅ SLAM Toolbox           (async_slam_toolbox_node)")
    print("  ✅ trust_map_solver       (/viz_map_data @ 1.25 Hz)")
    print("  ✅ HTTP 伺服器           (埠 8000)")
    print("\n🖱️  控制:")
    print("  📜 滾輪      → 縮放地圖")
    print("  🖱️  左鍵拖曳  → 移動視野")
    print("  🔘 右鍵      → 導航泡泡")
    print("\n按 Ctrl+C 停止所有服務")
    print("=" * 70 + "\n")
    
    # === 保持運行 ===
    def signal_handler(sig, frame):
        print("\n\n🛑 正在停止所有服務...")
        for p in processes:
            if p:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except:
                    pass
        time.sleep(1)
        print("✅ 所有服務已停止")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)

if __name__ == '__main__':
    main()
