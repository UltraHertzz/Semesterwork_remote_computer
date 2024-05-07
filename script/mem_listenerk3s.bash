#!/bin/bash
LOG_FILE="$HOME/Semesterwork"
LOG_FILE_NAME="memory_k3s.log"
mkdir -p "$(dirname "$LOG_FILE")"
if [ -f "$LOG_FILE/$LOG_FILE_NAME" ]; then
    rm "$LOG_FILE/$LOG_FILE_NAME"
    echo "log file deleted"
fi

while true; do
    echo Writing memory usage ...
    ps aux | awk '/k3s/ && !/awk/ {
    cpu_sum += $3
    mem_sum += $4
    }
    END {
    print "Total CPU used by Kubernetes processes: " cpu_sum "%"
    print "Total MEM used by Kubernetes processes: " mem_sum "%"
    }' >> $LOG_FILE/$LOG_FILE_NAME

    echo "$(date): Updated CPU and MEM usage by Kubernetes processes." >> $LOG_FILE/$LOG_FILE_NAME
    echo "$(date +"%s.%N"): Memory usage" >> $LOG_FILE/$LOG_FILE_NAME
    free -h >> $LOG_FILE/$LOG_FILE_NAME
    sleep 2
done