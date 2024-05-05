#!/bin/bash

# uncomment the following line to increase verbosity of the script
# main setup
TOPIC_NAME="/exec_time/controller_to_driver"
EXP_TIME=20

## uncomment from here
# Bare_metal version, run this script on the host machine while run relative script on the target machine(OrangePi)
LOG_FILE="$HOME/ros2_ws/log/runtime/bare_metal/topic/${TOPIC_NAME//\//_}.log"
mkdir -p "$(dirname "$LOG_FILE")"

if [ -f"$LOG_FILE" ];then
    rm "$LOG_FILE"
    echo "log file deleted"
fi
cd $HOME/ros2_ws
source install/setup.bash
echo "exec start time: $(date +"%s.%N")"
export ROS_DOMAIN_ID=30
MAX_ATTEMPTS=1000
attempt=0

while [ $attempt -lt $MAX_ATTEMPTS ]; do

    output=$(timeout 5 ros2 topic echo $TOPIC_NAME 2>&1 )
    
    if echo "$output" | grep -q "does not appear to be published yet\|Could not determine the type for the passed topic"; then
        echo "Warning detected, retrying..."
        ((attempt++))
    else
        echo "received data at time: $(date +"%s.%N")"
        break
    fi
    
    sleep 0.0001
done

if [ $attempt -eq $MAX_ATTEMPTS ]; then
    echo "Failed to capture data after $MAX_ATTEMPTS attempts."
fi
ros2 topic echo $TOPIC_NAME >> $LOG_FILE 2>&1 &
PID=$!

sleep $EXP_TIME
echo $EXP_TIME
kill $PID

echo "kill all ros2 process."
echo "exec end time: $(date +"%s.%N")"

# 