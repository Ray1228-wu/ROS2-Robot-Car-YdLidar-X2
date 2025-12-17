#  製作一台透過光學雷達實現定位與網頁指派的小車 (Building a Localization and Web-Controlled Robot Car using LiDAR)

## 專案概述 (Overview)

本文件旨在指導如何建構一個功能強大的分散式機器人系統，該系統以 **Raspberry Pi 4** 作為核心控制器，運行 **ROS2 Humble**，並整合 **光學雷達 (LiDAR)** 實現環境感知與**定位 (Localization)**。

**核心功能包括：**
1.  **環境感知與定位：** 使用 LiDAR 數據建構地圖，並利用地圖進行自我定位 (SLAM/AMCL)。
2.  **精確運動控制：** 整合帶有編碼器的馬達，實現精確的閉迴路 (Closed-loop) 速度控制。
3.  **分散式通訊：** 透過網路，將 Pi 上的 LiDAR 數據傳輸至 PC (WSL) 上的 **RViz2** 進行視覺化監控。
4.  **網頁控制/監控：** 透過安裝 **ROS Bridge** 套件，為實現網頁介面遠端指派任務和狀態監控做準備。

---

##  必備組件 (Required Components)

### 軟體 (Software)

* **Operating System:**
    * Raspberry Pi: **Ubuntu 22.04 LTS** (64-bit)
    * PC/WSL: **Ubuntu 22.04 LTS** (under Windows)
* **ROS Distribution:** **ROS 2 Humble Hawksbill**
* **Key Libraries/Tools:**
    * `python3-pip`, `numpy`, `opencv-python`
    * `ros-humble-desktop-full`, `ros-humble-desktop`
    * `ros-humble-nav2-bringup`, `ros-humble-slam-toolbox`, `ros-humble-ros2-control`
    * `ros-humble-ydlidar-ros2-driver`
    * `ros-humble-tf2-web-republisher`, `ros-humble-rosbridge-suite`

### 硬體 (Hardware)

| 零組件 (Component) | 規格/型號 (Spec/Model) | 作用 (Function) |
| :--- | :--- | :--- |
| **主控單元** | Raspberry Pi 4 Model B (4GB/8GB) | 運行 ROS2 核心控制與感知節點。 |
| **光學雷達 (LiDAR)** | YDLIDAR X2 (或其他相容型號) | 提供環境點雲數據，用於 SLAM/定位。 |
| **馬達與編碼器** | FIT0450 直流馬達 with Encoder | 提供動力；編碼器用於里程計 (Odometry)。 |
| **馬達驅動** | L298N Motor Driver | 雙路大電流直流馬達驅動，控制速度與方向。 |
| **電壓匹配** | 8-channel Bi-Directional Level Shifter | 轉換 Pi (3.3V) 與 L298N (5V/12V) 之間的邏輯電壓。 |
| **動力來源** | 兩顆 18650 鋰電池 | 為整個機器人提供動力 (通常串聯提供 $7.4V\sim 8.4V$)。 |
| **電源元件** | XT60, XL4015 降壓模組 | 電池連接與電壓穩壓。 |
| **PC/開發機** | 個人電腦 (PC + WSL/Ubuntu 22.04) | 遠端視覺化 (RViz2) 和程式開發。 |

---

## 軟體流程：Pi 主機環境設定 (Pre-installation - Pi)

此專案選用 ROS2 Humble，所以 Pi Imager 燒錄映像檔採用 **Ubuntu 22.04 ver**。

1.  確保系統是最新版本

    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```

2.  安裝語系支援 (ROS 2 要求)

    ```bash
    sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    source /etc/default/locale
    ```

3.  安裝基礎工具 (VSCode, Git, C++ 編譯環境所需)

    ```bash
    sudo apt install software-properties-common curl git unzip rsync build-essential tmux -y
    ```

    > **註：** 執行此步驟後，若有提示配置畫面，請採用他的預設按 OK。

4.  安裝 ROS 2 倉庫密鑰

    ```bash
    sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```

5.  添加 ROS 2 倉庫 (Humble for Ubuntu 22.04)

    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

6.  更新套件列表 (讓系統識別新倉庫)

    ```bash
    sudo apt update
    ```

7.  安裝 ROS 2 Desktop Full (包含所有 SLAM, Nav2 的 C++ 運行時依賴)

    ```bash
    sudo apt install -y \
      ros-humble-desktop-full
    ```

8.  專案核心功能套件 (選用)

    ```bash
    sudo apt install -y \
      ros-humble-desktop-full \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-ros2-control \
      ros-humble-rplidar-ros \
      python3-colcon-common-extensions \
      python3-rosdep
    ```

9.  設定 ROS 環境變數 (讓 ROS 指令在每次登入時生效)

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

10. Python 與視覺函式庫安裝

    ```bash
    sudo apt install python3-pip -y
    pip install \
      numpy \
      opencv-python
    ```
    > **註：** 採用他的預設按 OK。

11. 設定 Python 工具的 PATH

    * 步驟一：編輯 Shell 設定檔 `nano ~/.bashrc`
    * 步驟二：新增 PATH 環境變數

        ```bash
        export PATH=$PATH:$HOME/.local/bin
        ```

    * 步驟三：儲存並立即生效
        ```bash
        source ~/.bashrc
        ```

12. 測試 ROS 2 環境

    ```bash
    ros2 run demo_nodes_cpp talker
    ```

---

##  軟體流程：電腦安裝 ROS2 in WSL (Pre-installation - WSL)

1.  確定 WSL 版本是 ubuntu22.04

    ```bash
    wsl --install -d Ubuntu-22.04
    ```

2.  系統更新與基礎工具

    ```bash
    sudo apt update && sudo apt upgrade -y
    sudo apt install software-properties-common curl git build-essential -y
    ```

3.  設定語系

    ```bash
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    source /etc/default/locale
    ```

4.  設定 ROS 2 倉庫 (AMD64)

    ```bash
    sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

5.  更新並安裝 ROS 2 Desktop

    ```bash
    sudo apt update
    sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-pip
    ```

6.  安裝 Python 函式庫

    ```bash
    pip install numpy opencv-python
    ```

7.  環境設定

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "export PATH=\$PATH:\$HOME/.local/bin" >> ~/.bashrc
    source ~/.bashrc
    ```

8.  測試

    ```bash
    ros2 run rviz2 rviz2
    ```

## ROS Domain 通訊建立 (Pi & WSL)

* **WSL 網路設定：**
    * 確保 `~/.wslconfig` 檔案中包含 `networkingMode=mirrored`。
    * 執行 `wsl --shutdown` 重啟 WSL。
* **設定 ROS Domain ID (Pi & WSL)：**
    ```bash
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
    source ~/.bashrc
    ```
* **通訊測試：**
    * PC (WSL) 啟動 Listener：`ros2 run demo_nodes_py listener`
    * Pi 啟動 Talker：`ros2 run demo_nodes_cpp talker`
    * **預期結果：** PC 端應顯示接收到的訊息。

---

##  系統實作與接線 (Implementation)

### 1. 硬體接線與電源管理 (Hardware Connection & Power Management)



**電源電路：**
1.  **18650 電池組：** 兩顆 18650 串聯後 (約 $7.4V\sim 8.4V$) 透過 **XT60母端**輸出。
2.  **電源分流與穩壓：** 主電源從 XT60 分為兩路：
    * **第一路 (Pi 供電)：** 接入 **XL4015 降壓模組**。XL4015 輸出約 $5V$ 的穩定電壓，專門提供給 **Raspberry Pi 4** (5V 供電)。
    * **第二路 (馬達供電)：** 直接連接到 **L298N 驅動板**的馬達供電輸入端（未經 XL4015 降壓，直接使用電池電壓），並**建議在 L298N 的輸入端並聯大容量電解電容**（用於濾波和穩定馬達啟動時的瞬時電流）。
3.  **LiDAR & Encoder 供電與訊號路徑：**
    * **LiDAR 供電與訊號：** LiDAR 透過 **USB 連接線**直接從 **Pi 4B 獲得電力** (間接來自 XL4015 穩壓電源)，並同時透過 USB 傳輸數據。
    * **Encoder 供電與訊號：** 編碼器電源由 **L298N 驅動板** (通常為 5V 輸出) 供電。編碼器產生的訊號（通常為 5V 邏輯）透過 **8-channel Level Shifter** 降壓轉換後，連接到 **Raspberry Pi 4的 GPIO 接收端**。

**線路示意圖：**
![Screenshot 2025-12-16 114630](https://hackmd.io/_uploads/BJ6UUU0fWg.png)


**L298N 馬達驅動與 Pi 4B GPIO 接線 (BCM Pinout)：**

| Pi 4B GPIO (BCM) | 零組件 | 輸入端 | 功能 |
| :--- | :--- | :--- | :--- |
| **18** | **Motor L PWM** | L298n ENA | 左馬達速度 (PWM) |
| **14** | **Motor L Dir** | L298n IN1 | 左馬達方向控制 1 |
| **15** | **Motor L Dir** | L298n IN2 | 左馬達方向控制 2 |
| **22** | **Motor R PWM** | L298n ENB | 右馬達速度 (PWM) |
| **17** | **Motor R Dir** | L298n IN3 | 右馬達方向控制 1 |
| **27** | **Motor R Dir** | L298n IN4 | 右馬達方向控制 2 |
**2** | **Encoder L A** | 8Level Shifter A0 | 左馬達測距 | 
| **3** | **Encoder L B** | 8Level Shifter A1 | 左馬達測距  |
| **23** | **Encoder R A** | 8Level Shifter A6 | 右馬達測距  |
| **24** | **Encoder R B** | 8Level Shifter A7 | 右馬達測距  |
| **3.3V 針腳** | **Level Shifter LV** | VCCA &VCCB | 模組低壓側供電 |
| **5V 針腳** | **Level Shifter HV** | **麵包板和Encoder供電並聯** | 模組高壓側供電 (Encoder 邏輯電壓) |
| **GND** | **L298N, Encoder, Shifter** | GND | GND | 共同接地 |

麵包板上執行Y型分接以確保encoder輸出5V的電流不會流到
Pi上，並讓Pi可以順利收到來自encoder的邏輯電流，故將encoder端的VCC 和 Level Shifter HV  VCCA & VCCB 以及 Pi 5V VCC(pin 2 or 4) 做並聯


### 2. YDLIDAR 驅動安裝與 RViz 驗證

#### 2.1 YDLIDAR 驅動安裝 (Pi)

* **允許一般使用者讀取 USB 埠：**
    ```bash
    sudo usermod -a -G dialout $USER
    ```
* **設定 USB 規則 (`udev rules`)：**

    ```bash
    cd ~/ros2_ws/src/ydlidar_ros2_driver/startup
    chmod 777 ./*
    sudo sh initenv.sh
    ```

#### 2.2 YDLIDAR 參數調整

* **手動調整參數** 將設定檔規格改成 X2 規格：

    ```bash
    nano ~/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
    ```

    * **X2 規格核心參數：** `baudrate: 115200`，`lidar_type: 1`，`frame_id: laser_frame`。

#### 2.3 透過 RViz 驗證 Pi 和 Ydlidar 建立連接 (實踐)



* **步驟 1：測試掃描功能 (Pi 端)**

    ```bash
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py
    ```

* **步驟 2：啟動 RViz2 (PC 端)**

    ```bash
    rviz2
    ```

* **步驟 3：修正 RViz 設定**

    * 在 RViz 左側面板，找到 **Global Options**
    * 點擊 **Fixed Frame** 旁邊的 `map`
    * 把它改成跟您 yaml 檔裡的 `frame_id` 一樣，通常是 **`laser_frame`**
    * 點擊左下角的 **[Add]** 按鈕
    * 切換到 **[By Topic]** 分頁
    * 找到 **`/scan`**，點選下面的 **LaserScan**
    * 點擊 **[OK]**

    * **(如果還是沒看到紅點)：**
        * 在左側列表展開剛剛新增的 **LaserScan**
        * 把 **Reliability Policy** 的從 `Reliable` 改成 **`Best Effort`**。



### 3. 網頁投放跟節流 (WSL)

* **安裝 topic_tools (節流)：**
    ```bash
    sudo apt update
    sudo apt install ros-humble-topic-tools
    ```
* **安裝 Web 相關套件：**
    ```bash
    sudo apt install ros-humble-tf2-web-republisher
    sudo apt install ros-humble-rosbridge-suite
    ```

---

## Hardware (硬體配置)

本專案是一個複雜的機電整合系統，硬體配置細節如下：

| 零組件 (Component) | 規格/型號 (Spec/Model) | 作用 (Function) |
| :--- | :--- | :--- |
| **主控單元** | Raspberry Pi 4| 運行 ROS2 核心控制與感知節點。 |
| **光學雷達** | YDLIDAR X2 | 提供環境點雲數據，用於 SLAM/定位。 |
| **馬達與驅動** | L298N + FIT0450 with Encoder | 執行運動命令；編碼器用於里程計 (Odometry)。 |
| **電源管理** | 18650 (x2), XT60, XL4015 | 提供和穩壓馬達及 Pi 的主要動力。 |
| **電壓匹配** | 8-channel Bi-Directional Level Shifter | 橋接 Pi (3.3V) 和驅動板 (5V/12V) 之間的邏輯電壓。 |
| **PC/開發機** | 個人電腦 (PC + WSL/Ubuntu 22.04) | 遠端視覺化 (RViz2) 和程式開發。 |

###  硬體連線概覽



* **電源流向：** 18650 電池組 $\rightarrow$ XT60 $\rightarrow$ XL4015 (降壓) $\rightarrow$ L298N (馬達電源) & Level Shifter。
* **控制訊號：** Pi 4B (3.3V 邏輯) $\rightarrow$ Level Shifter $\rightarrow$ L298N (馬達 PWM/方向)。
* **反饋訊號：** 編碼器 (Encoder) $\rightarrow$ Level Shifter $\rightarrow$ Pi 4B GPIOs (用於讀取脈衝)。
* **感知與通訊：** LiDAR $\rightarrow$ Pi 4B USB；Pi 4B $\leftrightarrow$ PC (WSL) 透過 Wi-Fi/Ethernet 進行 ROS 通訊。

---

##  成果展示(Demo Video)

---



## 參考資料 (References)
https://hackaday.io/project/197642-rplidar-c1-with-raspberry-pi-4-and-ros2

https://medium.com/@jiayi.hoffman/prlidar-in-ros-2-docker-on-raspberry-pi-06086e968564

https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

http://www.robotbrigade.com/mappingRobot.php

https://docs.google.com/document/d/1gwkeCLW_RRi6tINub1Y3P05DJqnUuMJ5te2nC33V53c/edit?tab=t.0

https://www.youtube.com/watch?v=yu-TBDQx6pc&list=LL&index=3&t=519s

https://www.youtube.com/watch?v=qNdcXUEF7KU&list=LL&index=4

https://www.youtube.com/watch?v=ao13F-L_TAI&list=LL&index=5&t=1s

https://www.youtube.com/watch?v=uuslVHPjWS8&list=LL&index=7&t=1128s
