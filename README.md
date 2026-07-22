## 專案使用方式


### 執行 ROS 2 節點

每次重新開啟終端機後，先進入工作區並載入環境：
```bash
colcon build --symlink-install
```
```bash
source install/setup.bash
```
---
### 執行launch

設定 Arduino 序列埠權限：
```bash
sudo chmod 777 /dev/ttyUSB0
```
執行launch
```bash
ros2 launch hiwinduck_launch system.launch.py
```
---

#### 1. 啟動 RealSense 相機
```bash
ros2 launch realsense2_camera rs_launch.py
```
#### 2. 啟動 AprilTag
```bash
ros2 run yolo apriltag
```
#### 3. 啟動 YOLO
```bash
ros2 run yolo yolo_sub
```
#### 4. 啟動 HIWIN 手臂連線
```bash
ros2 run hiwin_libmodbus hiwinlibmodbus_server
```
#### 5. 轉換矩陣
```bash
ros2 run hiwin_example camera_flange_matrix2
```
#### 6. 執行手臂控制程式
```bash
ros2 run hiwin_example strategy_example
```
#### 7. 啟動力量感測器
1.開arduino

2.貼上force_kg 執行 

安裝 Python Serial 套件：

```bash
sudo apt update
sudo apt install -y python3-serial
```

設定 Arduino 序列埠權限：

```bash
sudo chmod 777 /dev/ttyUSB0
```

啟動 ROS 2 節點：

```bash
ros2 run arduino_bridge force_pub
```

執行力量感測節點時，不要開啟 Arduino Serial Monitor，否則序列埠會被占用，ROS 2 節點將無法讀取資料。

#### 8. 執行力感測監測
```bash
ros2 run insertion_monitor insertion_monitor_node
```

#### 9. 夾爪
docker內進資料夾
```bash
cd src/hiwin_gripper/HIWIN_XEG32/src/controller
```
執行夾爪程式
```bash
python3 Hiwin_API_pretest.py
```



---
### 常用指令
```bash
ros2 run <pkg_name> <entry_point>  //執行節點

ros2 pkg create <pkg_name> --build-type ament_cmake     # 創建新的 ROS 2 C++ 套件
ros2 pkg create <pkg_name> --build-type ament_python	  # 創建新的 ROS 2 Python 套件

sudo chmod 777 /dev/ttyACM0
```

---

### 1. 啟動 Docker 環境

先進入 Docker 資料夾：

建立映像：

```bash
. build.sh
```

開放顯示權限：

```bash
xhost +local:
```

啟動容器：

```bash
. run.sh
```

進入 ROS 2 工作區：

```bash
cd ~/work
```

---

### 2. 第一次使用：設定 RealSense 版本

進入 `realsense-ros`：

```bash
cd ~/work/src/realsense-ros
git fetch --tags
git checkout 4.57.7
```

如果先前編譯過其他版本，先清除快取：

```bash
cd ~/work
rm -rf build/realsense2_camera
rm -rf install/realsense2_camera
rm -rf log
```

重新編譯：

```bash
colcon build --symlink-install
```

這段通常只需要在第一次設定，或 RealSense 版本錯誤時執行。

---


## 建議啟動順序

```text
1. RealSense 相機
2. YOLO
3. HIWIN 手臂連線
4. 力量感測器
5. 手臂控制程式
```

每個 ROS 2 節點建議使用不同終端機執行。
