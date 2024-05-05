#! /bin/bash
cd $HOME/ros2_ws
source $HOME/ros2_ws/install/setup.bash
TOPIC_NAME="/cmd_vel"
TOPIC_LOG_PATH=${HOME}/ros2_ws/log/runtime/bare_matel

# check if the log file exists, if so, delete it
if [ -f "$TOPIC_LOG_PATH" ]; then
    rm "$TOPIC_LOG_PATH"
    echo "log file deleted"
fi
# check if the directory exists, if not, create it
mkdir -p "$(dirname "$TOPIC_LOG_PATH")"

export ROS_DOMAIN_ID=30
MAX_ATTEMPTS=1000
attempt=0
while true; do

    output=$(timeout 5 ros2 topic echo $TOPIC_NAME 2>&1 )
    
    if echo "$output" | grep -q "does not appear to be published yet\|Could not determine the type for the passed topic"; then
        echo "Warning detected, retrying..."
        ((attempt++))
    else
        echo "received data at time: $(date +"%s.%N")" >> ${LOG_FILE_PATH}/bare_matel_${TOPIC_NAME\\/\\_}.log
        break
    fi
    
    sleep 0.0001
done

ros2 topic echo $TOPIC_NAME >> $TOPIC_LOG_PATH/bare_matel_${TOPIC_NAME\\/\\_}.log 2>&1 