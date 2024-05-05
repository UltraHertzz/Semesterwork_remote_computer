#!/bin/bash
LOG_FILE="$HOME/Semesterwork"
mkdir -p "$(dirname "$LOG_FILE")"
if [ -f "$LOG_FILE/memory.log" ]; then
    rm "$LOG_FILE/memory.log"
    echo "log file deleted"
fi

while true; do
    echo Writing memory usage ...
    ps aux | awk '/ros2/ && !/awk/ {
    cpu_sum += $3
    mem_sum += $4
    }
    END {
    print "Total CPU used by ROS2 processes: " cpu_sum "%"
    print "Total MEM used by ROS2 processes: " mem_sum "%"
    }' >> $LOG_FILE/memory.log

    echo "$(date): Updated CPU and MEM usage by ROS2 processes." >> $LOG_FILE/memory.log
    echo "$(date +"%s.%N"): Memory usage" >> $LOG_FILE/memory.log
    free -h >> $LOG_FILE/memory.log
    sleep 2
done