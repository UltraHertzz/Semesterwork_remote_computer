#!/bin/bash

EXP_TIME=20
# Kubernetes version
TOPIC_NAME="/exec_time"
LOG_FILE="$HOME/ros2_ws/log/runtime/container/topic/${TOPIC_NAME//\//_}.log"
mkdir -p "$(dirname "$LOG_FILE")"

cd $HOME/ros2_ws/
if [ -f"$LOG_FILE" ];then
    rm "$LOG_FILE"
    echo "log file deleted"
fi
echo "kubectl deploy start time: $(date +"%s.%N")"
kubectl apply -f $HOME/ros2_ws/kube_config/controller.yaml
export ROS_DOMAIN_ID=30
while [ $(kubectl get pods | grep on-board-driver | grep 3/3 | wc -l) -eq 0 ]; do
    sleep 0.001
done
if [ $(ros2 topic list | grep $TOPIC_NAME | wc -l) -eq 1 ]; then
    echo "received topic time: $(date +"%s.%N")"
    ros2 topic echo $TOPIC_NAME >> $LOG_FILE 2>&1 &
    PID=$!
    sleep $EXP_TIME
else
    echo "topic is not exist."
fi

kill -SIGINT $PID
kubectl delete pods --all

echo "kill all kubernetes service process."
echo "exec end time: $(date +"%s.%N")"