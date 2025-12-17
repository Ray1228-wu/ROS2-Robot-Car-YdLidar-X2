// ==========================================
// 1. 初始化設定 (Initialization)
// ==========================================
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const scaleText = document.getElementById('scale-text');
const scaleLine = document.getElementById('scale-line');
const navBubble = document.getElementById('nav-bubble');

// 視圖狀態
let scale = 50; // 50 pixels = 1 meter
let offsetX = window.innerWidth / 2;
let offsetY = window.innerHeight / 2;
let isDragging = false;
let startDragX, startDragY;
let lastX, lastY;
let targetX = 0, targetY = 0;

// 機器人與地圖數據
let mapData = []; 
let robotPose = {x: 0, y: 0, theta: 0};

// 導航狀態
let navigationActive = false;
let navigationGoal = {x: 0, y: 0};
let motorPaused = false;
let encoderData = {x: 0, y: 0, theta: 0, distance: 0};  // 來自 PI 的編碼器數據

// ==========================================
// 2. 連接 ROS 2 (Connection)
// ==========================================
// 建立 ROS 連線物件 (預設 Port 9090)
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log('Connected to websocket server.');
    const statusEl = document.querySelector('.status');
    if(statusEl) {
        statusEl.innerText = "狀態: ROS 已連線";
        statusEl.style.color = "#00ff00"; 
    }
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    const statusEl = document.querySelector('.status');
    if(statusEl) {
        statusEl.innerText = "狀態: 連線錯誤 (檢查 rosbridge)";
        statusEl.style.color = "#ff0000";
    }
});

ros.on('close', () => {
    console.log('Connection closed.');
    const statusEl = document.querySelector('.status');
    if(statusEl) {
        statusEl.innerText = "狀態: 連線中斷";
        statusEl.style.color = "#ff0000";
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
    
    // 視角變換
    ctx.translate(offsetX, offsetY); 
    ctx.scale(scale, scale);
    
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
    ctx.rotate(robotPose.theta);

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
    requestAnimationFrame(draw);
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
    e.preventDefault();
    const worldX = (e.clientX - offsetX) / scale;
    const worldY = (e.clientY - offsetY) / scale;
    
    targetX = worldX;
    targetY = worldY;

    if(navBubble) {
        navBubble.style.left = `${e.clientX}px`;
        navBubble.style.top = `${e.clientY}px`;
        navBubble.classList.remove('hidden');
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