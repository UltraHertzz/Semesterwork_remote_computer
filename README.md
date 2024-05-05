# instruction file
This is the repository for navigation of a wheel robot, which should be run on remote Computer. Make sure that you have set up all necessary nodes on the wheel robot and has a WLAN to let them communicate with each other. 
## 1. Prerequest
for bare metal version, your machine should better to have the same setup as:
|Software|Version|
|---|-----|
| Linux | Ubuntu 22.04 LTS|
| ROS2 | Humble |
|Docker | -|
|Kubernetes(here k3s)|-|

# 2. bare metal
all tasks are run with ROS_DOMAIN_ID=30, ensure to set environment variable like `export ROS_DOMAIN_ID=30` before you want to check any topic information, otherwise you will get NOTHING useful from `ros2 topic list`.
### 1. navigation
```bash
# in ros2 workspace
colcon build
source install/setup.bash
ros2 launch car_nav navigation.launch.py # only lidar odometry and nav2 stack. you should make sure that there is another node that publish /scan(LaserScan), /odom(/Odometry)
```
### 2. slam
```bash
# in ros2 workspace
colcon build 
source install/setup.bash 
ros2 launch car_slam online_async_launch.py
```
# 3. containerized 
make sure that you are accessable for registry
```bash
# in ros2 workspace
kubectl apply -f kube_config/navigation.yaml
```
