# 🤖 YDLidar ROS2 機器人控制系統 - 完整使用指南

> **一體化文件**：初始化 + 使用 + 快速參考 + 故障排查 + 技術細節

**最後更新**：2025-12-16 | **版本**：2.0 (背景模式支援) | **狀態**：✅ 生產就緒

---

## 📑 快速導覽

- [快速開始 (30秒)](#-快速開始-30秒)
- [安裝與初始化](#-安裝與初始化)
- [使用指南](#-使用指南)
- [命令速查表](#-命令速查表)
- [故障排查](#-故障排查)
- [技術細節](#-技術細節)

---

## ⚡ 快速開始 (30秒)

### 背景模式啟動（推薦）✨

```bash
cd ~/ros2_ws/ros2_PI
python3 robot_control/start_robot_simple.py --background --wait 10
```

**預期輸出：**
```
✅ LIDAR 背景服務已啟動 (PID: 2678)
   進程將在背景持續運行...

🚀 後續步驟現在可以執行:
   1. 運行導航程式: python3 ~/ros2_ws/robot_control/navigation_logic.py
   2. 檢查訊息: ros2 topic list
```

### 前景模式啟動（簡單，阻塞式）

```bash
python3 robot_control/start_robot_simple.py
# 按 Ctrl+C 停止
```

---

## 🛠️ 安裝與初始化

### 步驟 1：編譯項目（只需一次）

```bash
cd ~/ros2_ws/ros2_PI
colcon build
```

**檢查點：**
- ✅ 編譯完成（無紅色錯誤）
- ✅ `install/` 資料夾已生成

### 步驟 2：檢查硬體

```bash
# 檢查 LIDAR USB 連接
ls -la /dev/ttyUSB*

# 檢查 GPIO 權限
ls -la /sys/class/gpio/

# 檢查 Python 環境
python3 --version  # 應為 3.8+
```

**連接清單：**
- LIDAR：USB 轉串口（通常 `/dev/ttyUSB0`）
- 編碼器：GPIO 2, 3, 23, 24
- 馬達驅動：GPIO 14, 15, 17, 18, 22, 27

### 步驟 3：設置 USB 權限

```bash
# 給予 LIDAR 設備讀寫權限
sudo chmod 666 /dev/ttyUSB0

# 或加入 dialout 群組（永久）
sudo usermod -a -G dialout $USER
```

### 步驟 4：驗證 ROS2 環境

```bash
# 檢查 ROS2 是否正確安裝
echo $ROS_DISTRO  # 應顯示 "humble"

# 檢查 ROS2 命令可用
ros2 --version
```

---

## 📚 使用指南

### 背景模式 vs. 前景模式

| 特性 | 背景模式 | 前景模式 |
|------|---------|---------|
| **主程式阻塞** | ❌ 否 | ✅ 是 |
| **可執行後續步驟** | ✅ 是 | ❌ 否 |
| **LIDAR 運行時間** | 手動管理 | 主程式結束時停止 |
| **適合場景** | 複雜應用、自動化 | 簡單測試、偵錯 |
| **停止方式** | `kill` / `killall` | `Ctrl+C` |

### 背景模式詳細用法

#### 基礎啟動

```bash
# 使用默認等待時間 (3.0 秒)
    python3 robot_control/start_robot_simple.py --background

# 指定等待時間（給 Raspberry Pi 更多初始化時間）
    python3 robot_control/start_robot_simple.py --background --wait 5

# 使用自訂參數檔
    python3 robot_control/start_robot_simple.py --background --wait 4 --params ./ydlidar.yaml
```

#### 後續步驟（同一終端）

```bash
# 主程式返回後立即執行
$ python3 robot_control/start_robot_simple.py --background
✅ LIDAR 背景服務已啟動 (PID: 2678)

$ ros2 topic list              # ← 立即可執行！
$ ros2 topic echo /scan -n 1   # 查看掃描數據
$ python3 navigation_logic.py  # 執行導航邏輯
```

#### 多終端協作

```bash
# 終端 1：啟動 LIDAR (背景模式)
$ python3 robot_control/start_robot_simple.py --background --wait 4

# 終端 2：監控日誌（可選）
$ tail -f ~/ros2_ws/robot_control/logs/start_robot_launch.log

# 終端 3：執行應用邏輯
$ python3 navigation_logic.py
$ python3 encoder_reader.py
$ python3 motor_driver.py
```

### 前景模式詳細用法

```bash
# 啟動（一直運行直到 Ctrl+C）
python3 start_robot_simple.py

# 使用自訂參數檔
python3 start_robot_simple.py --params ./ydlidar.yaml

# 停止（任何時候按 Ctrl+C）
^C  ← 會自動清理資源並結束
```

---

## 🔍 命令速查表

### 啟動命令

```bash
# 背景模式 (推薦)
python3 robot_control/start_robot_simple.py --background --wait 4

# 前景模式
python3 start_robot_simple.py

# 自訂參數
python3 robot_control/start_robot_simple.py --background --wait 5 --params /path/to/file.yaml
```

### 日誌與監控

```bash
# 即時監控日誌
tail -f ~/ros2_ws/robot_control/logs/start_robot_launch.log

# 查看最後 20 行
tail -20 ~/ros2_ws/robot_control/logs/start_robot_launch.log

# 搜索錯誤
grep -i "error\|fail" ~/ros2_ws/robot_control/logs/start_robot_launch.log

# 查看完整日誌
cat ~/ros2_ws/robot_control/logs/start_robot_launch.log
```

### ROS2 診斷命令

```bash
# 查看所有節點
ros2 node list

# 查看所有主題
ros2 topic list

# 查看節點詳細信息
ros2 node info /ydlidar_ros2_driver_node

# 監聽掃描數據
ros2 topic echo /scan -n 5

# 監聽點雲數據
ros2 topic echo /point_cloud -n 1

# 查看坐標變換
ros2 topic echo /tf | head -20

# 視覺化 TF 樹
ros2 run tf2_tools view_frames
```

### 進程管理

```bash
# 查看 LIDAR 相關進程
ps aux | grep ydlidar_ros2_driver

# 溫和停止背景服務
killall -f 'robot_control_launch.py'

# 強制停止（如溫和停止無效）
killall -9 ydlidar_ros2_driver_node

# 查看進程樹
pstree | grep ros2

# 使用 systemctl（如已配置為服務）
systemctl status lidar-service
```

### 執行後續應用

```bash
# 編碼器讀取
python3 encoder_reader.py

# 馬達驅動
python3 motor_driver.py

# 導航邏輯
python3 navigation_logic.py

# 安全控制
python3 safety_controller.py
```

---

## 🗂️ 重要檔案位置

```
~/ros2_ws/
├── robot_control/                              # 主控制目錄
│   ├── start_robot_simple.py                   # 主啟動腳本
│   ├── README.md                               # 本文件
│   ├── logs/
│   │   └── start_robot_launch.log              # 運行日誌
│   ├── ydlidar.yaml                            # 本地參數備份
│   ├── encoder_reader.py                       # 編碼器讀取模組
│   ├── motor_driver.py                         # 馬達驅動模組
│   ├── navigation_logic.py                     # 導航邏輯
│   └── safety_controller.py                    # 安全控制
├── src/
│   └── ydlidar_ros2_driver/
│       ├── launch/
│       │   └── robot_control_launch.py         # ROS2 Launch 檔案
│       └── params/
│           └── ydlidar.yaml                    # 主參數檔
├── install/                                     # 編譯產物
└── build/                                       # 編譯中間檔案
```

---

## ⚠️ 故障排查

### 常見問題

#### ❌ 進程立即退出，看到「LIDAR 連接失敗」

**原因：** LIDAR 未連接或 USB 權限不足

**解決：**
```bash
# 1. 檢查 USB 連接
ls -la /dev/ttyUSB*

# 2. 授予權限
sudo chmod 666 /dev/ttyUSB0

# 3. 重新啟動
python3 robot_control/start_robot_simple.py --background
```

#### ❌ 「等待超時」或節點初始化慢

**原因：** Raspberry Pi 處理能力有限，3 秒不夠

**解決：**
```bash
# 增加等待時間到 5 秒
python3 robot_control/start_robot_simple.py --background --wait 5
```

#### ❌ 後續命令無反應，沒有看到 `/scan` 主題

**原因：** ROS2 環境變數未加載

**解決：**
```bash
# 在運行任何 ros2 命令前，先設置環境
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

#### ❌ 日誌檔找不到或為空

**原因：** 後台執行緒讀寫延遲，或路徑錯誤

**解決：**
```bash
# 等待 2-3 秒後查看
sleep 3 && cat ~/ros2_ws/robot_control/logs/start_robot_launch.log

# 或使用即時監控
tail -f ~/ros2_ws/robot_control/logs/start_robot_launch.log
```

#### ❌ 「無法執行後續步驟」，程式一直等待

**原因：** 沒有使用 `--background` 參數，處於前景模式

**解決：**
```bash
# 停止當前程式
Ctrl+C

# 使用背景模式重新啟動
python3 robot_control/start_robot_simple.py --background --wait 4
```

#### ❌ YAML 參數檔錯誤

**原因：** 參數檔路徑錯誤或格式不正確

**解決：**
```bash
# 檢查參數檔是否存在
cat ~/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml

# 驗證 YAML 語法
python3 -c "import yaml; yaml.safe_load(open('ydlidar.yaml'))"

# 使用默認參數檔
python3 robot_control/start_robot_simple.py --background
```

### 系統診斷步驟

```bash
# 步驟 1：驗證 ROS2 安裝
$ ros2 --version
ROS 2 Humble

# 步驟 2：檢查編譯
$ ls ~/ros2_ws/install/ydlidar_ros2_driver
# 應該看到 lib/, share/ 等資料夾

# 步驟 3：驗證硬體
$ ls /dev/ttyUSB*
/dev/ttyUSB0

# 步驟 4：啟動並檢查
$ python3 robot_control/start_robot_simple.py --background
✅ LIDAR 背景服務已啟動 (PID: XXXX)

# 步驟 5：驗證主題
$ ros2 topic list | grep -E "scan|point_cloud"
/point_cloud
/scan
```

---

## 📊 技術細節

### 背景模式實現原理

**執行流程：**

```
主程式啟動
    ↓
subprocess.Popen() 啟動 ros2 launch (非阻塞)
    ↓
建立後台執行緒讀取進程輸出 (無阻塞)
    ↓
主執行緒 sleep(wait_time)
    ↓
檢查子進程是否仍在執行
    ↓
打印 "LIDAR 背景服務已啟動 (PID: XXXX)"
    ↓
主程式返回 ✅
    
後台：
  └─ 子進程 (ros2 launch) 繼續執行
  └─ 執行緒持續讀取輸出並寫入日誌
  └─ 直到 kill 命令或程式錯誤
```

**進程生命週期：**

```
背景模式：
  主程式 ──► subprocess.Popen() ──► 主程式返回 (4秒)
             │
             ├─► 後台執行緒 ──► 讀取輸出 ──► 寫入日誌 ──► (無限期運行)
             │
             └─► 子進程 ──► ros2 launch ──► LIDAR 驅動 ──► (無限期運行)

前景模式：
  主程式 ──► subprocess.Popen() ──► poll() 循環 ──► 主程式阻塞
             │
             └─► 子進程 ──► ros2 launch ──► LIDAR 驅動 ──► (主程式結束時停止)
```

### 參數說明

| 參數 | 型別 | 預設值 | 說明 |
|------|------|--------|------|
| `--background` | boolean | False | 啟用背景模式 |
| `--wait` | float | 3.0 | 背景模式下等待節點初始化時間（秒） |
| `--params` | string | 預設路徑 | 指定 YAML 參數檔路徑 |

### LIDAR 參數說明

```yaml
# ~/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
baudrate: 115200              # LIDAR 波特率
sample_rate: 3                # 採樣率（支援 3, 4）
fixed_resolution: true        # 固定分辨率模式
frequency: 5.5                # 掃描頻率 (Hz)
lidar_type: 1                 # 類型：1 = 三角測量
isSingleChannel: true         # 單通道模式
port: /dev/ttyUSB0            # LIDAR 設備檔
```

### ROS2 主題映射

| 主題 | 訊息類型 | 來源 | 發布率 | 說明 |
|------|---------|------|--------|------|
| `/scan` | LaserScan | LIDAR | 5.5 Hz | 2D 雷射掃描數據 |
| `/point_cloud` | PointCloud2 | LIDAR | 5.5 Hz | 3D 點雲數據 |
| `/tf` | Transform | TF Publisher | 10 Hz | 座標變換 |
| `/tf_static` | Transform | TF Publisher | 1 Hz | 靜態座標變換 |

### 坐標系統

```
base_link (機器人底盤)
    ↓
laser_frame (LIDAR 座標系)
    └─ 距離 base_link 0.02m (Z 軸高度)
```

---

## 🎯 最佳實踐

### ✅ 推薦做法

```bash
# 開發階段：背景模式 + 監控
終端 1: python3 robot_control/start_robot_simple.py --background --wait 5
終端 2: tail -f logs/start_robot_launch.log &
終端 3: python3 navigation_logic.py

# 測試階段：驗證各模組
終端 1: python3 robot_control/start_robot_simple.py --background
終端 2: python3 encoder_reader.py
終端 3: python3 motor_driver.py

# 生產環境：使用 systemd 服務或其他進程管理器
# (可選配置，此處省略)
```

### ❌ 應避免

- ❌ 多次執行 `start_robot_simple.py`（會衝突）
- ❌ 不檢查進程就假設初始化完成
- ❌ 直接 `kill -9` 而不清理資源
- ❌ 混淆背景模式和前景模式
- ❌ 忽視日誌檔中的警告訊息

---

## 📞 版本資訊

| 項目 | 值 |
|------|-----|
| **更新日期** | 2025-12-16 |
| **版本** | 2.0 (Background Mode) |
| **ROS2 版本** | Humble |
| **Python 版本** | 3.8+ |
| **平台** | Raspberry Pi 4B / Linux |
| **狀態** | ✅ 生產就緒 |

---

## 📖 參考資料

### 相關檔案

- **主啟動腳本**：[start_robot_simple.py](./start_robot_simple.py)
- **參數檔**：[ydlidar.yaml](./ydlidar.yaml)
- **日誌檔**：[logs/start_robot_launch.log](./logs/start_robot_launch.log)

### 外部資源

- [ROS2 官方文檔](https://docs.ros.org/en/humble/)
- [YDLidar ROS2 驅動](https://github.com/YDLIDAR/YDLIDAR-ROS2)
- [ROS2 命令行工具](https://docs.ros.org/en/humble/Concepts/Basic/About-ROS-2.html)

---

## 💡 提示與技巧

### 性能優化

- **增加 LIDAR 掃描率**：修改 `frequency: 5.5` 為更高值（最多 10 Hz）
- **降低 CPU 佔用**：減少後台主題發布率，使用 `scan_throttled`
- **改進導航精度**：調整 `sample_rate` 為 4（更多採樣點）

### 日誌管理

```bash
# 清理舊日誌
echo > ~/ros2_ws/robot_control/logs/start_robot_launch.log

# 備份日誌
cp ~/ros2_ws/robot_control/logs/start_robot_launch.log ~/logs_backup_$(date +%Y%m%d).log
```

### 快速故障修復

```bash
# 一鍵重啟 LIDAR
killall -9 ydlidar_ros2_driver_node 2>/dev/null
sleep 2
python3 robot_control/start_robot_simple.py --background

# 檢查系統狀態
echo "=== ROS2 節點 ===" && ros2 node list && \
echo "=== 主題訊息 ===" && ros2 topic list | head -10 && \
echo "=== 進程狀態 ===" && ps aux | grep ydlidar
```

---

## ❓ 常見問題 (FAQ)

**Q: 背景模式和前景模式哪個更好？**  
A: 背景模式更靈活，適合複雜應用。前景模式更簡單，適合測試。

**Q: 如何在多個終端執行不同的應用？**  
A: 先用背景模式啟動 LIDAR，再在其他終端執行應用即可。

**Q: LIDAR 掉線怎麼辦？**  
A: 檢查 USB 連接、重新授予權限、重啟程式。

**Q: 如何設置開機自動啟動？**  
A: 可配置 systemd 服務或 crontab（進階）。

---

## 🔐 安全與權限

### 所需權限

```bash
# LIDAR USB 設備
/dev/ttyUSB0 (666)

# GPIO (如使用馬達控制)
/sys/class/gpio/ (666)

# 日誌檔目錄
~/ros2_ws/robot_control/logs/ (755)
```

### 設置權限

```bash
# 一鍵設置所有權限
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
```

---

## 🎓 學習路徑

**新手：** 快速開始 → 基本命令 → 故障排查  
**進階：** 背景模式詳細用法 → 多應用協作 → 性能優化  
**專家：** 技術細節 → 源碼修改 → 自訂擴展

---

**最後更新：2025-12-16**  
**作者**：Robot Control Team  
**狀態**：✅ 完整、經驗證、生產就緒

---

## 🚀 下一步

現在你已經了解了系統。選擇一個開始吧：

1. **快速開始** → 執行 `python3 robot_control/start_robot_simple.py --background --wait 4`
2. **深入學習** → 閱讀完整的技術細節部分
3. **故障排查** → 如果遇到問題，查看故障排查章節
4. **執行應用** → 運行 `navigation_logic.py` 等應用模組

祝你使用愉快！🎉
