# hiwinduck

/////初始化/////
1.下載主專案
git clone https://github.com/zhen0929829959/hiwinduck.git

下載docker
git clone https://github.com/errrr0501/ubuntu22.04_ros2.git
. build.sh 
xhost +local:
. run.sh

cd src

2.下載相機套件
git clone https://github.com/realsenseai/realsense-ros.git

3.下載手臂套件
git clone --recurse-submodules https://github.com/tku-iarc/Hiwin_libmodbus.git
cd ..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

4.相機版本更改
cd ~/work/src/realsense-ros
git fetch --tags
git tag --list | tail -20
git checkout 4.57.7

（若報錯）
cd ~/work
rm -rf build/realsense2_camera
rm -rf install/realsense2_camera
rm -rf log

colcon build --symlink-install



/////執行ros2專案/////
#開相機
colcon build --symlink-install
source install/setup.bash
ros2 launch realsense2_camera rs_align_depth_launch.py 

#yolo
colcon build
source install/setup.bash
ros2 run yolo yolo_sub

#手臂連線
colcon build
source install/setup.bash
ros2 run hiwin_libmodbus hiwinlibmodbus_server

#run hiwin
colcon build
source install/setup.bash
ros2 run hiwin_example strategy_example 


#git note
1.git add .
2.git commit -m "xxx"
3.git push
