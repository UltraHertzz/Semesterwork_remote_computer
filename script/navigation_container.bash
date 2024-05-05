#! /bin/bash
cd $HOME/ros2_ws
source $HOME/ros2_ws/install/setup.bash
DATE=$(date +%Y-%m-%d-%H-%M-%S)
LOG_FILE_PATH=${HOME}/ros2_ws/log/runtime/bare_matel
mkdir -p "$(dirname "$LOG_FILE_PATH")"
TOPIC_NAME="/cmd_vel"
mkdir -p "$(dirname "$LOG_FILE_PATH")"
TOPIC_LOG_PATH=${HOME}/ros2_ws/log/runtime/bare_matel/bare_matel_${TOPIC_NAME\\/\\_}_${DATE}.log
if [ "$ROS_DOMAIN_ID" != "30" ]; then
    export ROS_DOMAIN_ID=30
fi
echo "exec start time: $(date+"%s.%N")" >> ${LOG_FILE_PATH}/bare_matel_nav_${DATE}.log
ros2 launch navigation navigation.launch.py >> ${LOG_FILE_PATH} 2>&1 &
ros2 topic echo /scan >> ${TOPIC_LOG_PATH} 2>&1 &
