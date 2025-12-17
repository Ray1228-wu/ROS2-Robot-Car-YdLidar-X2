# 🤖 機器人導航系統（Robot Monitor Navigation System）
## 快速啟動指南（README）

---

## 🚀 步驟 1：啟動 ROS 2 系統（WSL 終端）

```bash
cd ros2_PC/          # 進入工作區目錄
python3 start_all.py # 使用相對路徑自動啟動
```

> 💡 **相對路徑優點**  
> 複製整個資料夾到任何位置都能直接執行  
> 無需修改絕對路徑 `/home/ray/ros2_PC`

### 預期輸出

- ✅ rosbridge_websocket running (port 9090)
- ✅ throttle node running (0.8 Hz)
- ✅ SLAM running
- ✅ trust_map_solver running
- ✅ HTTP server running (port 8000)

---

## 🌐 步驟 2：開啟網頁（任何瀏覽器）

```
http://localhost:8000
```

✔ 應看到即時 **SLAM 地圖** 與 **機器人箭頭**

---

## 🧭 步驟 3：測試導航功能

### 🅰 模式 A：自動導航（右鍵點擊）

1. 右鍵點擊地圖任意位置 → 出現「前往」泡泡  
2. 點擊泡泡 → 底部導航欄滑出  
   - ⏸ 暫停（橙色）
   - ✕ 取消（紅色）
3. 發送 `/nav_goal` 至 PI
4. 暫停 → 發送 `PAUSE` 到 `/motor_control`
5. 繼續 → 發送 `RESUME`
6. 取消 → 發送 `CANCEL`

---

### 🅱 模式 B：手動控制（鍵盤 WASD）

> 頁面需取得焦點

| 按鍵 | 動作 |
|----|----|
| W | 前進（0.3 m/s） |
| S | 後退（0.3 m/s） |
| A | 左轉（0.3 rad/s） |
| D | 右轉（0.3 rad/s） |
| Space | 暫停 |
| 放開 WASD | 停止 |

📤 發佈至 `/manual_control`（與導航控制獨立）

```text
FORWARD|0.3
LEFT|0.3
STOP
```

---

## 🧠 步驟 4：驗證路徑記憶（可選）

```bash
ros2 topic echo /viz_map_data
```

- `walls: X` 數字應逐漸增加
- 表示牆體歷史資料已永久保存

---

## 🧩 系統組件清單

### ✅ 已完成（WSL 端）

- Web UI（index.html）
- JavaScript 邏輯（map_viewer.js）
- 路徑記憶（trust_map_solver.py）
- 系統啟動（start_all.py）
- 測試工具（test_navigation.py）

### ⏳ 待實作（PI 端）

- GPIO 馬達驅動
- 編碼器讀取
- 導航演算法
- 參考：`pi_motor_controller_template.py`

---

## 🛠 常見問題（Troubleshooting）

- **導航欄不顯示**：檢查 console、nav-bar 元素、JS 是否載入
- **WASD 無效**：確認頁面焦點、PI 是否訂閱 `/manual_control`
- **ROS 無反應**：確認 `ROS_DOMAIN_ID=30`

---

## 🔑 主要 ROS 2 話題

### 📤 Publish（Web → PI）

- `/nav_goal`（PoseStamped）
- `/motor_control`（PAUSE / RESUME / CANCEL）
- `/manual_control`（FORWARD / LEFT / STOP）

### 📥 Subscribe（PI → Web）

- `/encoder_state`（Twist）
- `/robot_status`（String）

---

🎉 **祝使用愉快！🤖**
