# 🤖 機器人導航系統 — 快速啟動指南
Robot Monitor Navigation System – Quick Start

---

## 🚀 步驟 1：啟動 ROS 2 系統（WSL 終端）

```bash
cd ros2_PC/          # 進入工作區目錄
python3 start_all.py # 使用相對路徑自動啟動
```

> 💡 **相對路徑優點**  
> 複製整個資料夾到任何位置都能直接執行  
> 無需修改 `/home/ray/ros2_PC`

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

✔ 應看到 **即時 SLAM 地圖** 與 **機器人箭頭**

---

## 🧭 步驟 3：測試導航功能

### 🅰 模式 A：自動導航（右鍵點擊）

1. **右鍵點擊地圖**
   - 出現「前往」泡泡

2. **點擊泡泡**
   - 導航欄由底部滑出
   - 按鈕：⏸ 暫停（橙）｜✕ 取消（紅）
   - 發送 `/nav_goal` 到 PI

3. **暫停**
   - 發送 `PAUSE` → `/motor_control`

4. **繼續**
   - 發送 `RESUME` → `/motor_control`

5. **取消**
   - 發送 `CANCEL` → `/motor_control`

---

### 🅱 模式 B：手動控制（鍵盤 WASD）

> 頁面需取得焦點

| 鍵盤 | 動作 |
|----|----|
| W | 前進（0.3 m/s） |
| S | 後退 |
| A | 左轉（0.3 rad/s） |
| D | 右轉 |
| Space | 暫停 |
| 放開 WASD | 停止 |

📤 發佈到 `/manual_control`（與導航獨立）

```text
FORWARD|0.3
LEFT|0.3
STOP
```

---

🎉 **祝使用愉快！**
