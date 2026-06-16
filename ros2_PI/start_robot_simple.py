#!/usr/bin/python3
"""
統一版機器人啟動器（僅 launch 模式，含日誌與安全退出）

用法:
    - 啟動 YDLidar + 靜態 TF（ROS2 Launch）:
        python3 start_robot_simple.py
    - 指定參數檔（覆寫預設 params/ydlidar.yaml）:
        python3 start_robot_simple.py --params ../src/ydlidar_ros2_driver/params/ydlidar.yaml
"""


import subprocess
import sys
import os
from pathlib import Path
import signal
import time
from threading import Thread
import argparse


def cleanup_existing():
    """Stop existing launch/driver processes to avoid duplicates."""
    patterns = [
        "robot_control_launch.py",
        "ros2 launch ydlidar_ros2_driver",
        "ydlidar_ros2_driver_node",
        "navigation_logic.py",
        "encoder_reader.py",
        "motor_driver.py",
    ]
    for pat in patterns:
        subprocess.run(["pkill", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stream_output(proc: subprocess.Popen, log_file, node_name: str = ""):
    """持續讀取子進程輸出，寫入檔案。"""
    prefix = f"[{node_name}] " if node_name else ""
    for raw in proc.stdout:  # iterates line by line
        try:
            text = raw.decode(errors='replace')
        except Exception:
            text = str(raw)
        try:
            log_file.write(f"{prefix}{text}")
            log_file.flush()
        except ValueError:
            # 檔案已關閉，停止寫入
            break
        if not node_name:  # 只有 LIDAR 輸出到終端
            print(text, end='')

def run_launch_mode(workspace_root: Path, params_path: str | None = None, background: bool = True, wait_time: float = 3.0):
    """
    啟動 ROS2 Launch (LIDAR + 靜態 TF)
    
    Args:
        workspace_root: 工作區根目錄
        params_path: 參數檔路徑 (覆寫預設)
        background: True = 背景啟動，主程式繼續執行下一步
                   False = 前景阻塞，等待 launch 結束
        wait_time: 背景模式下，等待 ROS2 節點就緒的時間 (秒)
    """
    # 嘗試多個可能的 setup 腳本位置
    possible_setups = [
        workspace_root / 'install' / 'setup.bash',  # 本地 build
        Path('/opt/ros/humble/setup.bash'),  # 全局安裝
    ]
    
    setup_script = None
    for p in possible_setups:
        if p.exists():
            setup_script = p
            break
    
    if not setup_script:
        print("❌ 錯誤: 找不到 ROS2 setup 腳本")
        for p in possible_setups:
            print(f"   {p}")
        print("\n請先執行以下命令編譯項目:")
        print(f"  cd {workspace_root}")
        print("  colcon build")
        sys.exit(1)

    log_dir = workspace_root / 'robot_control' / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file_path = log_dir / 'start_robot_launch.log'

    print("=" * 60)
    print("🤖 機器人驅動程式 (launch) 啟動")
    print("=" * 60)
    mode_text = "背景模式 (主程式繼續執行後續步驟)" if background else "前景模式 (阻塞等待)"
    print(f"\n啟動: YDLidar + 靜態 TF via ROS2 Launch [{mode_text}]")
    print("按 Ctrl+C 停止，日誌:" , log_file_path)
    print()

    # 若指定了參數檔，透過 launch 參數覆寫
    params_arg = ""
    if params_path:
        params_arg = f" params_file:={params_path}"
        print(f"使用參數檔: {params_path}")

    # 啟動前先清理可能遺留的相同進程
    cleanup_existing()

    # 找到 launch 檔案
    launch_file = workspace_root / 'install' / 'ydlidar_ros2_driver' / 'share' / 'ydlidar_ros2_driver' / 'launch' / 'robot_control_launch.py'
    if not launch_file.exists():
        print(f"❌ 錯誤: 找不到 launch 檔案")
        print(f"   {launch_file}")
        sys.exit(1)
    
    cmd = f"bash -i -c 'source /opt/ros/humble/setup.bash && source {setup_script} && ros2 launch {launch_file}{params_arg}'"

    with log_file_path.open('a', encoding='utf-8') as lf:
        lf.write("\n=== Launch start {} ===\n".format(time.strftime('%Y-%m-%d %H:%M:%S')))
        lf.flush()

        proc = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        t = Thread(target=stream_output, args=(proc, lf, ""), daemon=True)
        t.start()

        # ===== 新增：背景模式支援 =====
        if background:
            # 背景模式：等待初始化後立即返回
            print(f"⏳ 等待 ROS2 節點初始化... ({wait_time:.1f}秒)")
            time.sleep(wait_time)
            
            # 檢查進程是否仍在運行
            if proc.poll() is not None:
                print(f"\n⚠️  警告: Launch 進程提前退出 (代碼: {proc.returncode})")
                print(f"日誌檔: {log_file_path}")
                sys.exit(1)
            
            print(f"✅ LIDAR 背景服務已啟動 (PID: {proc.pid})")
            print(f"   進程將在背景持續運行，可在另一終端檢查日誌:")
            print(f"   tail -f {log_file_path}")
            print()
            return proc  # 返回進程對象供調用者管理
        
        # ===== 前景模式：原有阻塞邏輯 =====
        try:
            while True:
                ret = proc.poll()
                if ret is not None:
                    break
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n⚠️  偵測到中斷，正在結束進程...")
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("強制結束進程 (SIGKILL)")
                os.killpg(proc.pid, signal.SIGKILL)

        t.join(timeout=2)
        lf.write("=== Launch end (code={}) ===\n".format(proc.returncode))
        lf.flush()

        if proc.returncode not in (0, 130):
            print(f"\n❌ 啟動失敗 (代碼: {proc.returncode})")
            print(f"日誌檔: {log_file_path}")
            print("建議檢查：")
            print("1. cd ~/ros2_ws && colcon build")
            print("2. which ros2 / source /opt/ros/<distro>/setup.bash")
            print("3. ls /dev/ttyUSB* 檢查雷達連接與權限 (需 dialout 群組)")
            sys.exit(1)
        else:
            print(f"\n✅ 啟動結束 (code={proc.returncode})，日誌: {log_file_path}")

def start_control_nodes(workspace_root: Path, control_mode: str = 'auto'):
    """
    啟動機器人控制節點
    
    Args:
        workspace_root: 工作區根目錄
        control_mode: 'auto' = 自動模式 (motor_driver)
                     'manual' = 手動模式 (manual_motor_driver)
                     'both' = 同時啟動兩者 (不推薦，GPIO 衝突)
    """
    control_dir = workspace_root / 'robot_control'
    log_dir = control_dir / 'logs'
    log_file_path = log_dir / 'robot_nodes.log'
    
    # 根據模式選擇啟動哪些節點
    base_nodes = [
        ('navigation_logic.py', '導航邏輯'),
        ('encoder_reader.py', '編碼器讀取'),
    ]
    
    if control_mode == 'manual':
        nodes = base_nodes + [('manual_motor_driver.py', '手動馬達控制')]
    elif control_mode == 'both':
        nodes = base_nodes + [
            ('motor_driver.py', '馬達驅動'),
            ('manual_motor_driver.py', '手動馬達控制'),
        ]
    elif control_mode == 'manager':
        # 使用模式管理器，自動切換手動/自動驅動
        nodes = base_nodes + [('motor_mode_manager.py', '馬達模式管理器')]
    else:  # auto (default)
        nodes = base_nodes + [('motor_driver.py', '馬達驅動')]
    
    processes = []
    
    with log_file_path.open('a', encoding='utf-8') as lf:
        lf.write(f"\n{'='*60}\n")
        lf.write(f"Control nodes start {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        lf.write(f"{'='*60}\n")
        lf.flush()
        
        for script_name, display_name in nodes:
            script_path = control_dir / script_name
            if not script_path.exists():
                print(f"⚠️  警告: {script_name} 不存在，跳過")
                continue
            
            print(f"▶️  啟動 {display_name} ({script_name})...")
            
            env = os.environ.copy()
            env['PYTHONNOUSERSITE'] = '1'  # 避免 NumPy 2.x 衝突
            
            proc = subprocess.Popen(
                [sys.executable, str(script_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,
            )
            
            # 啟動執行緒讀取輸出
            t = Thread(target=stream_output, args=(proc, lf, display_name), daemon=True)
            t.start()
            
            processes.append((proc, display_name))
            time.sleep(0.8)  # 錯開啟動時間
        
        # 等待節點初始化
        print(f"\n⏳ 等待節點初始化... (2秒)")
        time.sleep(2)
        
        # 檢查進程是否正常運行
        failed = []
        for proc, name in processes:
            if proc.poll() is not None:
                failed.append((name, proc.returncode))
        
        if failed:
            print(f"\n⚠️  以下節點啟動失敗:")
            for name, code in failed:
                print(f"   - {name} (代碼: {code})")
            print(f"\n查看日誌: tail -20 {log_file_path}")
            # 移除失敗的進程
            processes = [(p, n) for p, n in processes if p.poll() is None]
        
        return processes

def main():
    parser = argparse.ArgumentParser(description="啟動 YDLidar + 靜態 TF")
    parser.add_argument("--params", dest="params", help="參數檔路徑（覆寫預設）", default=None)
    parser.add_argument("--background", dest="background", action="store_true", 
                        help="背景模式：LIDAR 後台運行，主程式繼續執行後續步驟 (預設: False)")
    parser.add_argument("--wait", dest="wait_time", type=float, default=3.0,
                        help="背景模式下等待節點初始化時間 (秒，預設: 3.0)")
    parser.add_argument("--control-mode", dest="control_mode", 
                        choices=['auto', 'manual', 'both', 'manager'], default='manager',
                        help="控制模式: auto=自動, manual=手動, manager=智能切換(推薦), both=兩者(不推薦)")
    args = parser.parse_args()

    script_dir = Path(__file__).parent
    workspace_root = script_dir  # ros2_PI 本身就是工作區根目錄
    
    lidar_proc = run_launch_mode(
        workspace_root, 
        params_path=args.params, 
        background=args.background,
        wait_time=args.wait_time
    )
    
    # ===== 後續步驟：啟動機器人控制節點 (背景模式才會執行) =====
    if args.background and lidar_proc is not None:
        print("=" * 60)
        print("🚀 啟動機器人控制節點...")
        print("=" * 60)
        
        control_nodes = start_control_nodes(workspace_root, control_mode=args.control_mode)
        
        if control_nodes:
            print("\n✅ 所有系統已就緒！")
            print("\n運行中的服務:")
            print(f"   - LIDAR 驅動 (PID: {lidar_proc.pid})")
            for proc, name in control_nodes:
                print(f"   - {name} (PID: {proc.pid})")
            print("\n📋 管理命令:")
            print("   查看日誌: tail -f ~/ros2_ws/robot_control/logs/start_robot_launch.log")
            print("   檢查節點: ros2 node list")
            print("   檢查主題: ros2 topic list")
            print("   停止所有: pkill -f 'robot_control/(navigation_logic|encoder_reader|motor_driver|robot_control_launch).py'")
        else:
            print("\n⚠️  控制節點啟動失敗，請檢查日誌")
        print("=" * 60)

if __name__ == '__main__':
    main()
