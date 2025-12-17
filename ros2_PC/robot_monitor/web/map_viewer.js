// ==========================================
// 1. 初始化設定 (Initialization)
// ==========================================
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const scaleText = document.getElementById('scale-text');
const scaleLine = document.getElementById('scale-line');
const navBubble = document.getElementById('nav-bubble');
const manualToggleBtn = document.getElementById('manual-toggle');
const manualPanel = document.getElementById('manual-panel');
const keyElements = {
    'W': document.querySelector('.key-w'),
    'A': document.querySelector('.key-a'),
    'S': document.querySelector('.key-s'),
    'D': document.querySelector('.key-d'),
};
const rosStatusEl = document.getElementById('ros-status');

// 視圖狀態
let scale = 50; // 50 pixels = 1 meter
let offsetX = window.innerWidth / 2;
let offsetY = window.innerHeight / 2;
let isDragging = false;
let startDragX, startDragY;
let lastX, lastY;
let targetX = 0, targetY = 0;

// LiDAR 額外偏角（供前端臨時覆蓋，單位：度；來源：URL 參數 ?yaw= 或 localStorage.lidarYawDeg）
const getYawDegFromEnv = () => {
    try {
        const url = new URL(window.location.href);
        const yawParam = url.searchParams.get('yaw');
        if (yawParam !== null && yawParam !== '') return parseFloat(yawParam);
    } catch {}
    try {
        const y = localStorage.getItem('lidarYawDeg');
        if (y !== null && y !== '') return parseFloat(y);
    } catch {}
    return 0;
};
let lidarYawDeg = getYawDegFromEnv();
if (!Number.isFinite(lidarYawDeg)) lidarYawDeg = 0;
let lidarYawOffsetRad = lidarYawDeg * Math.PI / 180;
console.log(`[MapViewer] LiDAR 前端偏角: ${lidarYawDeg}° (${lidarYawOffsetRad.toFixed(4)} rad)`);

// 水平反轉設定（URL 參數 ?mirrorX=1/0 或 localStorage.mirrorX）
function parseBoolish(v, def=false) {
    if (v === null || v === undefined || v === '') return def;
    const s = String(v).toLowerCase();
    if (s === '1' || s === 'true' || s === 'yes') return true;
    if (s === '0' || s === 'false' || s === 'no') return false;
    return def;
}
function getMirrorXFromEnv() {
    try {
        const url = new URL(window.location.href);
        const p = url.searchParams.get('mirrorX');
        if (p !== null) return parseBoolish(p, true);
    } catch {}
    try {
        const ls = localStorage.getItem('mirrorX');
        if (ls !== null) return parseBoolish(ls, true);
    } catch {}
    return true; // 需求：目前需要水平反轉，預設為開啟
}
let mirrorX = getMirrorXFromEnv();
console.log(`[MapViewer] 水平反轉 mirrorX=${mirrorX}`);

// 更新偏角（同時更新 localStorage 與畫面顯示）
const updateYawUI = () => {
    const el = document.getElementById('yaw-display');
    if (el) el.innerText = `${Math.round(lidarYawDeg)}°`;
};
const clampDeg = (d) => {
    // 夾在 [-180, 180]
    let v = d;
    while (v > 180) v -= 360;
    while (v < -180) v += 360;
    return v;
};
const setLidarYawDeg = (d) => {
    lidarYawDeg = clampDeg(d);
    lidarYawOffsetRad = lidarYawDeg * Math.PI / 180;
    try { localStorage.setItem('lidarYawDeg', String(lidarYawDeg)); } catch {}
    updateYawUI();
    console.log(`[MapViewer] 更新 LiDAR 偏角: ${lidarYawDeg}° (${lidarYawOffsetRad.toFixed(4)} rad)`);
};
updateYawUI();

// 羅盤畫布與拖曳控制
const compass = document.getElementById('yaw-compass');
const btnRST = document.getElementById('yaw-reset');
if (btnRST) btnRST.addEventListener('click', () => setLidarYawDeg(0));

let draggingCompass = false;
function getCompassAngleDegFromEvent(e) {
    if (!compass) return lidarYawDeg;
    const rect = compass.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const mx = e.clientX;
    const my = e.clientY;
    // 以螢幕座標轉換為數學座標（y 向上），取得從 +X（右）逆時針的角度
    const ang = Math.atan2(cy - my, mx - cx); // rad, 0=右, +90=上
    const deg = ang * 180 / Math.PI;          // deg
    // 轉為我們的 yaw 定義：0=上（北），正為逆時針
    const yawFromNorth = deg - 90; // 上=0°
    return clampDeg(yawFromNorth);
}

function drawCompass() {
    if (!compass) return;
    const ctx2 = compass.getContext('2d');
    const w = compass.width, h = compass.height;
    ctx2.clearRect(0,0,w,h);
    const cx = w/2, cy = h/2, r = Math.min(w,h)/2 - 8;
    // 外圈
    ctx2.beginPath();
    ctx2.arc(cx, cy, r, 0, Math.PI*2);
    ctx2.strokeStyle = '#666';
    ctx2.lineWidth = 2;
    ctx2.stroke();
    // 上方為「正前」的標記刻度
    ctx2.beginPath();
    ctx2.moveTo(cx, cy - r);
    ctx2.lineTo(cx, cy - r + 10);
    ctx2.strokeStyle = '#999';
    ctx2.lineWidth = 2;
    ctx2.stroke();
    // 指針（以 yawDeg 繪製；0=上=正前，正為逆時針）
    const angDeg = lidarYawDeg + 90; // 轉回 0=右 參考框架
    const ang = angDeg * Math.PI / 180;
    const ex = cx + Math.cos(ang) * (r - 6);
    const ey = cy - Math.sin(ang) * (r - 6); // y 向上
    ctx2.beginPath();
    ctx2.moveTo(cx, cy);
    ctx2.lineTo(ex, ey);
    ctx2.strokeStyle = '#d32f2f';
    ctx2.lineWidth = 3;
    ctx2.stroke();
    // 中心點
    ctx2.beginPath();
    ctx2.arc(cx, cy, 3, 0, Math.PI*2);
    ctx2.fillStyle = '#d32f2f';
    ctx2.fill();
}

if (compass) {
    compass.addEventListener('mousedown', (e) => {
        draggingCompass = true;
        setLidarYawDeg(getCompassAngleDegFromEvent(e));
    });
    window.addEventListener('mousemove', (e) => {
        if (!draggingCompass) return;
        setLidarYawDeg(getCompassAngleDegFromEvent(e));
    });
    window.addEventListener('mouseup', () => {
        draggingCompass = false;
    });
}

// 機器人與地圖數據
let mapData = []; 
let robotPose = {x: 0, y: 0, theta: 0};
// 頁面視圖固定偏移：取消（僅讓 LiDAR 點陣隨機器人旋轉）
const viewRotationOffset = 0;
// 固定箭頭旋轉偏移（畫面基準，逆時針 90°）
const arrowRotationOffset = -Math.PI / 2;

// 導航狀態
let navigationActive = false;
let navigationGoal = {x: 0, y: 0};
let motorPaused = false;
let encoderData = {x: 0, y: 0, theta: 0, distance: 0};  // 來自 PI 的編碼器數據

// 馬達方向跟蹤（用於箭頭指向）
let motorDirection = -50 * Math.PI / 180; // 初始逆時針 50 度，符合實際車頭方向

// ==========================================
// 2. 連接 ROS 2 (Connection)
// ==========================================
// 建立 ROS 連線物件 (預設 Port 9090)
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log('Connected to websocket server.');
    if (rosStatusEl) {
        rosStatusEl.innerText = "狀態: ROS 已連線";
        rosStatusEl.style.color = "#00ff00"; 
    }
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    if (rosStatusEl) {
        rosStatusEl.innerText = "狀態: 連線錯誤 (檢查 rosbridge)";
        rosStatusEl.style.color = "#ff0000";
    }
});

ros.on('close', () => {
    console.log('Connection closed.');
    if (rosStatusEl) {
        rosStatusEl.innerText = "狀態: 連線中斷";
        rosStatusEl.style.color = "#ff0000";
    }
});

// 訂閱話題 /viz_map_data
// [改進] 添加接收速率限制，避免JavaScript事件隊列堆積
let lastMapUpdateTime = 0;
const MAP_UPDATE_THROTTLE = 80; // 毫秒，最多每80ms更新一次 (1.25 Hz)

const mapListener = new ROSLIB.Topic({
    ros : ros,
    name : '/viz_map_data',
    messageType : 'std_msgs/String'
});

mapListener.subscribe((message) => {
    try {
        // [改進] 客戶端節流：即使網頁接收很快，也限制更新頻率
        const now = Date.now();
        if (now - lastMapUpdateTime < MAP_UPDATE_THROTTLE) {
            return; // 跳過本次更新
        }
        lastMapUpdateTime = now;
        
        const data = JSON.parse(message.data);
        robotPose = data.robot;
        mapData = data.map;
        
        // 更新左上角文字
        const elDistX = document.getElementById('dist-x');
        const elDistY = document.getElementById('dist-y');
        const elTheta = document.getElementById('robot-theta');
        if(elDistX) elDistX.innerText = robotPose.x.toFixed(2);
        if(elDistY) elDistY.innerText = robotPose.y.toFixed(2);
        if(elTheta) elTheta.innerText = (robotPose.theta * 180 / Math.PI).toFixed(1);
        
        // 日誌：顯示接收頻率
        console.log(`Map updated: ${mapData.length} points`);
    } catch (e) {
        console.error("JSON Parse Error:", e);
    }
});

// ==========================================
// 說明按鈕與對話框
// ==========================================
const helpBtn = document.getElementById('help-btn');
const helpModal = document.getElementById('help-modal');
const helpClose = document.getElementById('help-close');

if (helpBtn && helpModal && helpClose) {
    helpBtn.addEventListener('click', () => {
        helpModal.classList.remove('hidden');
    });
    
    helpClose.addEventListener('click', () => {
        helpModal.classList.add('hidden');
    });
    
    // 點擊外層背景也可以關閉
    helpModal.addEventListener('click', (e) => {
        if (e.target === helpModal) {
            helpModal.classList.add('hidden');
        }
    });
}

// ==========================================
// ROS Topic 發佈（導航目標和馬達控制）
// ==========================================

// 發佈導航目標
const navGoalPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/nav_goal',
    messageType: 'geometry_msgs/PoseStamped'
});

// 發佈馬達控制信號
const motorControlPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/motor_control',
    messageType: 'std_msgs/String'
});

// 發佈手動控制信號 (WASD 鍵盤控制)
const manualControlPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/manual_control',
    messageType: 'std_msgs/String'
});

// 發佈馬達模式（manual/auto），供後端選擇驅動檔
const motorModePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/motor_mode',
    messageType: 'std_msgs/String'
});

function enableManualMode() {
    const msg = new ROSLIB.Message({ data: 'manual' });
    motorModePublisher.publish(msg);
    console.log('已切換到手動模式');
}

function disableManualMode() {
    const msg = new ROSLIB.Message({ data: 'auto' });
    motorModePublisher.publish(msg);
    console.log('已切換到自動模式');
}

// 訂閱編碼器數據 (來自 PI)
const encoderTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/encoder_state',
    messageType: 'geometry_msgs/Twist'
});

encoderTopic.subscribe((message) => {
    encoderData.x = message.linear.x;
    encoderData.y = message.linear.y;
    encoderData.theta = message.angular.z;
    // 編碼器數據已接收（僅用於內部計算，不顯示）
});

// 訂閱機器人狀態
const robotStateTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_status',
    messageType: 'std_msgs/String'
});

robotStateTopic.subscribe((message) => {
    console.log('機器人狀態:', message.data);
    updateNavigationBar(message.data);
});

// ==========================================
// 3. 繪圖核心循環 (Render Loop)
// ==========================================
function draw() {
    // 確保 canvas 尺寸正確
    if (canvas.width !== window.innerWidth || canvas.height !== window.innerHeight) {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // ==========================================
    // A. 繪製地圖點 (牆壁)
    // ==========================================
    ctx.save();
    
    // 視角變換 + 以機器人位置為樞紐旋轉地圖（箭頭保持朝上）
    ctx.translate(offsetX, offsetY);
    ctx.scale(scale, scale);
    if (mirrorX) {
        // 全域水平反轉（不影響箭頭，箭頭獨立繪製）
        ctx.scale(-1, 1);
    }
    ctx.translate(robotPose.x, robotPose.y);
    // 改回負角旋轉以維持正確左右方向（Canvas 正角為順時針）
    ctx.rotate(-(robotPose.theta + lidarYawOffsetRad));
    ctx.translate(-robotPose.x, -robotPose.y);
    
    ctx.fillStyle = 'black';
    const GRID_SIZE = 0.1; 
    
    // 加上 try-catch 防止畫圖畫到一半資料壞掉導致卡死
    try {
        if (mapData && mapData.length > 0) {
            mapData.forEach(point => {
                // 1. 設定顏色
                if (point.s >= 4) {
                ctx.fillStyle = 'black'; // 牆壁
            } else if (point.s === 0) {
                ctx.fillStyle = '#ffffff'; // 純白色 (空氣)
            } else {
                ctx.fillStyle = 'gray'; // 其他情況 (保險起見)
            }
            
            // 2. 畫格子 (稍微畫大一點點 +0.02 避免縫隙)
            ctx.fillRect(point.x * GRID_SIZE, point.y * GRID_SIZE, GRID_SIZE + 0.02, GRID_SIZE + 0.02);
            });
        }
    } catch(err) {
        console.error("Drawing error:", err);
    }
    
    ctx.restore();

    // ==========================================
    // B. 繪製機器人 (Google Maps 風格：藍色導航箭頭)
    // 位置：LiDAR 座標位置（robotPose.x, robotPose.y）
    // ==========================================
    ctx.save();
    
    // 機器人在屏幕座標系上的位置
    // = (地圖座標 * 縮放 + 偏移)
    const robotScreenX = robotPose.x * scale + offsetX;
    const robotScreenY = robotPose.y * scale + offsetY;
    
    ctx.translate(robotScreenX, robotScreenY);
    // 箭頭固定朝向畫面正上（逆時針 90° 偏移）
    ctx.rotate(arrowRotationOffset);

    // 箭頭大小 (隨縮放，但限制最小/最大尺寸)
    const arrowSize = Math.min(Math.max(0.25 * scale, 8), 40); // 0.25m 基準，8-40px 範圍

    ctx.beginPath();
    // 1. 箭頭尖端 (前方)
    ctx.moveTo(arrowSize, 0); 
    // 2. 箭頭右後方
    ctx.lineTo(-arrowSize * 0.6, arrowSize * 0.7);
    // 3. 箭頭底部中心 (稍微內凹，營造立體感)
    ctx.lineTo(-arrowSize * 0.2, 0);
    // 4. 箭頭左後方
    ctx.lineTo(-arrowSize * 0.6, -arrowSize * 0.7);
    
    ctx.closePath(); // 自動連回尖端

    // 填充顏色 (Google Maps 藍)
    ctx.fillStyle = '#4285F4'; 
    // 加入陰影讓它浮起來
    ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
    ctx.shadowBlur = 10;
    ctx.shadowOffsetX = 2;
    ctx.shadowOffsetY = 2;
    ctx.fill();

    // 白色邊框 (讓它在深色或淺色地圖上都明顯)
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();

    // 畫一個中心圓點 (可選，模擬軸心)
    ctx.beginPath();
    ctx.arc(0, 0, 4, 0, 2 * Math.PI);
    ctx.fillStyle = 'white';
    ctx.fill();

    ctx.restore();
    
    updateScaleIndicator();
    // 繪製左下角羅盤
    drawCompass();
    requestAnimationFrame(draw);
}

// 將螢幕座標轉換為世界座標（考慮以機器人為旋轉樞紐）
function screenToWorld(screenX, screenY) {
    let vx = (screenX - offsetX) / scale;
    const vy = (screenY - offsetY) / scale;
    if (mirrorX) {
        // 與 draw() 的 ctx.scale(-1,1) 對應：螢幕座標 x 取負
        vx = -vx;
    }
    const px = vx - robotPose.x;
    const py = vy - robotPose.y;
    const theta = robotPose.theta + lidarYawOffsetRad;
    const cos = Math.cos(theta);
    const sin = Math.sin(theta);
    // 逆變換：旋轉 +theta（對應 draw() 的 -theta）
    const rx = cos * px - sin * py;
    const ry = sin * px + cos * py;
    return { x: rx + robotPose.x, y: ry + robotPose.y };
}

// 更新比例尺
function updateScaleIndicator() {
    if(!scaleLine || !scaleText) return;
    let unitDist = 1; 
    if (scale < 20) unitDist = 5;
    if (scale < 5) unitDist = 10;
    
    const pixelLength = unitDist * scale;
    scaleLine.style.width = `${pixelLength}px`;
    scaleText.innerText = `${unitDist} m`;
}

// ==========================================
// 4. 事件監聽 (Interaction)
// ==========================================

// 初始觸發一次
draw();

// --- 縮放 (滾輪) ---
canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomIntensity = 0.1;
    const direction = e.deltaY > 0 ? -1 : 1;
    const factor = 1 + (zoomIntensity * direction);
    
    const newScale = scale * factor;
    if (newScale > 5 && newScale < 300) {
        scale = newScale;
    }
}, { passive: false });

// --- 拖曳 (滑鼠左鍵) ---
canvas.addEventListener('mousedown', (e) => {
    if (e.button === 0) { 
        isDragging = true;
        startDragX = e.clientX;
        startDragY = e.clientY;
        lastX = offsetX;
        lastY = offsetY;
        if(navBubble) navBubble.classList.add('hidden');
    }
});

window.addEventListener('mousemove', (e) => {
    if (isDragging) {
        const dx = e.clientX - startDragX;
        const dy = e.clientY - startDragY;
        offsetX = lastX + dx;
        offsetY = lastY + dy;
    }
});

window.addEventListener('mouseup', () => {
    isDragging = false;
});

// --- 右鍵點擊跳出泡泡 ---
canvas.addEventListener('contextmenu', (e) => {
    if (manualEnabled) return; // 手動模式時鎖定導航泡泡
    e.preventDefault();
    const world = screenToWorld(e.clientX, e.clientY);
    targetX = world.x;
    targetY = world.y;

    if(navBubble) {
        navBubble.style.left = `${e.clientX}px`;
        navBubble.style.top = `${e.clientY}px`;
        navBubble.classList.remove('hidden');
    }
});

// --- 鍵盤控制 (WASD 前後左右，Space 暫停) ---
const keyState = {};
let manualLoopTimer = null;
let lastSentCommand = null;
let lastSendTime = 0;
const MANUAL_LOOP_MS = 100; // 10 Hz
let manualEnabled = false;
let modeSwitchPending = false; // 切換等待期間封鎖手動控制
const MODE_SWITCH_DELAY_MS = 700; // 模式切換緩衝時間（毫秒）

const updateKeyVisuals = () => {
    Object.entries(keyElements).forEach(([k, el]) => {
        if (!el) return;
        if (keyState[k]) {
            el.classList.add('active');
        } else {
            el.classList.remove('active');
        }
    });
};

const pickCommandFromKeys = () => {
    // W/S/A/D 優先級
    if (keyState['W']) return 'FORWARD|0.3';
    if (keyState['S']) return 'BACKWARD|0.3';
    if (keyState['A']) return 'LEFT|0.3';
    if (keyState['D']) return 'RIGHT|0.3';
    return 'STOP';
};

const publishManualCommand = (cmd) => {
    const msg = new ROSLIB.Message({ data: cmd });
    manualControlPublisher.publish(msg);
    lastSentCommand = cmd;
    lastSendTime = Date.now();
    // 只有 W 前進更新箭頭方向，S/A/D 不影響；逆時針 50 度偏移
    if (cmd === 'FORWARD|0.3') motorDirection = -50 * Math.PI / 180;
    console.log(`手動控制: ${cmd}`);
};

const startManualLoop = () => {
    if (manualLoopTimer) return;
    manualLoopTimer = setInterval(() => {
        const cmd = pickCommandFromKeys();
        // 規則：按住鍵時每 0.1s 發送；STOP 只在狀態改變時發送一次
        if (cmd !== 'STOP') {
            publishManualCommand(cmd);
        } else if (lastSentCommand !== 'STOP') {
            publishManualCommand('STOP');
        }
    }, MANUAL_LOOP_MS);
};

const stopManualLoop = () => {
    if (manualLoopTimer) {
        clearInterval(manualLoopTimer);
        manualLoopTimer = null;
    }
};

window.addEventListener('keydown', (e) => {
    const key = e.key.toUpperCase();
    if (!manualEnabled || modeSwitchPending) return; // 切換期間不接受鍵盤控制
    if (!['W','A','S','D'].includes(key)) return;
    keyState[key] = true;
    updateKeyVisuals();
    startManualLoop();
});

window.addEventListener('keyup', (e) => {
    const key = e.key.toUpperCase();
    if (!manualEnabled || modeSwitchPending) return; // 切換期間不接受鍵盤控制
    if (!['W','A','S','D'].includes(key)) return;
    keyState[key] = false;
    updateKeyVisuals();
    // 若所有鍵都放開，發送 STOP 並停止迴圈
    const anyPressed = Object.values(keyState).some(v => v);
    if (!anyPressed) {
        publishManualCommand('STOP');
        stopManualLoop();
    }
});

// 手動模式切換按鈕
if (manualToggleBtn) {
    manualToggleBtn.addEventListener('click', () => {
        if (!manualPanel) return;
        // 進入切換等待
        modeSwitchPending = true;
        manualToggleBtn.disabled = true;
        manualToggleBtn.innerText = '切換中…';

        if (!manualEnabled) {
            // 切到手動：先發 'manual'，等待訂閱生效後再開放手動
            motorModePublisher.publish(new ROSLIB.Message({ data: 'manual' }));
            setTimeout(() => {
                modeSwitchPending = false;
                manualEnabled = true;
                // 更新本地 UI
                manualPanel.classList.toggle('hidden', !manualEnabled);
                manualToggleBtn.classList.toggle('active', manualEnabled);
                manualToggleBtn.innerText = '手動模式 ON';
                manualToggleBtn.disabled = false;
            }, MODE_SWITCH_DELAY_MS);
        } else {
            // 切回自動：先發 STOP，再發 'auto'，等待退出完成
            publishManualCommand('STOP');
            motorModePublisher.publish(new ROSLIB.Message({ data: 'auto' }));
            // 重置鍵態與循環
            Object.keys(keyState).forEach(k => keyState[k] = false);
            updateKeyVisuals();
            stopManualLoop();
            if (navBubble) navBubble.classList.add('hidden');

            setTimeout(() => {
                modeSwitchPending = false;
                manualEnabled = false;
                // 更新本地 UI
                manualPanel.classList.toggle('hidden', !manualEnabled);
                manualToggleBtn.classList.toggle('active', manualEnabled);
                manualToggleBtn.innerText = '手動模式';
                manualToggleBtn.disabled = false;
            }, MODE_SWITCH_DELAY_MS);
        }
    });
}

// 視窗失焦或頁面隱藏時立即 STOP，符合 watchdog 安全規則
window.addEventListener('blur', () => {
    if (!manualEnabled) return;
    publishManualCommand('STOP');
    Object.keys(keyState).forEach(k => keyState[k] = false);
    updateKeyVisuals();
    stopManualLoop();
});
document.addEventListener('visibilitychange', () => {
    if (document.hidden && manualEnabled) {
        publishManualCommand('STOP');
        Object.keys(keyState).forEach(k => keyState[k] = false);
        updateKeyVisuals();
        stopManualLoop();
    }
});

// --- 點擊「前往」泡泡 ---
if(navBubble) {
    navBubble.addEventListener('click', (e) => {
        e.stopPropagation();
        
        // 發送導航目標到 ROS
        const goal = new ROSLIB.Message({
            header: {
                frame_id: 'map'
            },
            pose: {
                position: {x: targetX, y: targetY, z: 0},
                orientation: {x: 0, y: 0, z: 0, w: 1}
            }
        });
        
        navGoalPublisher.publish(goal);
        navigationActive = true;
        navigationGoal = {x: targetX, y: targetY};
        
        console.log(`導航目標: (${targetX.toFixed(2)}, ${targetY.toFixed(2)})`);
        navBubble.classList.add('hidden');
        
        // 顯示導航欄
        showNavigationBar();
    });
}

// ==========================================
// 導航欄控制函數
// ==========================================

function showNavigationBar() {
    const navBar = document.getElementById('nav-bar');
    if(navBar) {
        navBar.classList.remove('hidden');
        motorPaused = false;
        updatePauseButtonState();
    }
}

function updatePauseButtonState() {
    const pauseBtn = document.getElementById('pause-btn');
    if(pauseBtn) {
        if(motorPaused) {
            pauseBtn.innerText = '▶ 繼續';
            pauseBtn.style.backgroundColor = '#4CAF50';
        } else {
            pauseBtn.innerText = '⏸ 暫停';
            pauseBtn.style.backgroundColor = '#ff9800';
        }
    }
}

// 暫停按鈕
const pauseBtn = document.getElementById('pause-btn');
if(pauseBtn) {
    pauseBtn.addEventListener('click', () => {
        motorPaused = !motorPaused;
        
        // 發送馬達控制信號
        const controlMsg = new ROSLIB.Message({
            data: motorPaused ? 'PAUSE' : 'RESUME'
        });
        motorControlPublisher.publish(controlMsg);
        
        console.log(motorPaused ? '暫停導航' : '繼續導航');
        updatePauseButtonState();
    });
}

// 取消按鈕
const cancelBtn = document.getElementById('cancel-btn');
if(cancelBtn) {
    cancelBtn.addEventListener('click', () => {
        navigationActive = false;
        motorPaused = false;
        
        // 發送取消信號
        const cancelMsg = new ROSLIB.Message({
            data: 'CANCEL'
        });
        motorControlPublisher.publish(cancelMsg);
        
        console.log('導航已取消');
        
        // 隱藏導航欄
        const navBar = document.getElementById('nav-bar');
        if(navBar) {
            navBar.classList.add('hidden');
        }
    });
}

function updateNavigationBar(status) {
    const statusBar = document.getElementById('nav-status');
    const distanceBar = document.getElementById('nav-distance');
    
    if(!statusBar) return;
    
    // 解析狀態字串: "NAVIGATING|x|y|distance" 或 "IDLE|0|0|0"
    const parts = status.split('|');
    const state = parts[0];
    const targetX = parseFloat(parts[1]) || 0;
    const targetY = parseFloat(parts[2]) || 0;
    const distance = parseFloat(parts[3]) || 0;
    
    // 狀態映射
    const stateMap = {
        'NAVIGATING': '導航中',
        'IDLE': '待機',
        'PAUSED': '暫停',
        'ARRIVED': '已到達',
        'ERROR': '錯誤'
    };
    
    const displayState = stateMap[state] || state;
    statusBar.innerText = `狀態: ${displayState}`;
    
    // 更新距離顯示
    if(distanceBar) {
        if(state === 'NAVIGATING' || state === 'PAUSED') {
            distanceBar.innerText = `距離目標: ${distance.toFixed(2)} m`;
        } else {
            distanceBar.innerText = `距離目標: --`;
        }
    }
}