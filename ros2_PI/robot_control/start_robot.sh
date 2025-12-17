#!/bin/bash
#
# 機器人驅動啟動指令
# 使用方式: bash start_robot.sh
# 或: chmod +x start_robot.sh && ./start_robot.sh
#

set -e  # 發生錯誤時立即退出

# 取得腳本所在目錄
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=========================================="
echo "🤖 機器人驅動程式啟動"
echo "=========================================="
echo ""
echo "工作區路徑: $WORKSPACE_ROOT"
echo ""

# 檢查 ROS2 setup 腳本是否存在
SETUP_SCRIPT="$WORKSPACE_ROOT/install/setup.bash"
if [ ! -f "$SETUP_SCRIPT" ]; then
    echo "❌ 錯誤: 找不到 ROS2 setup 腳本"
    echo "   $SETUP_SCRIPT"
    echo ""
    echo "請先執行以下命令編譯項目:"
    echo "  cd $WORKSPACE_ROOT"
    echo "  colcon build"
    exit 1
fi

echo "✓ 加載 ROS2 環境..."
source "$SETUP_SCRIPT"

echo ""
echo "=========================================="
echo "✅ 啟動機器人完整系統"
echo "=========================================="
echo ""
echo "啟動中的模組:"
echo "  • YDLidar 掃描驅動"
echo "  • 編碼器讀取器"
echo "  • 馬達驅動器"
echo "  • 安全控制器"
echo "  • 導航邏輯"
echo ""
echo "按 Ctrl+C 停止所有系統"
echo "=========================================="
echo ""

# 啟動 launch 檔案
ros2 launch ydlidar_ros2_driver robot_control_launch.py
