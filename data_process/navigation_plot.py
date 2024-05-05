import matplotlib.pyplot as plt
import re
import datetime
import numpy as np

def parse_log_file(log_file_path):
    timestamps = []
    cpu_usages = []
    mem_usages = []
    first_time = None
    
    with open(log_file_path, 'r') as file:
        content = file.readlines()
    
    for i, line in enumerate(content):
        if 'Total CPU used by ROS2 processes:' in line:
            try:

                cpu_usage = float(line.split(':')[1].strip().strip('%')) 
            except ValueError:
                cpu_usage = 0
            try:
                mem_usage = float(content[i+1].split(':')[1].strip().strip('%'))
            except ValueError:
                mem_usage = 0 
            try:
                timestamp = float(content[i+3].split(':')[0].strip())
            except ValueError as e:
                print(f"Warning: Unable to parse timestamp from line {i}: {time_str}")
            if not first_time:
                first_time = timestamp
            seconds_since_start = (timestamp - first_time)
            
            timestamps.append(seconds_since_start)
            cpu_usages.append(cpu_usage)
            mem_usages.append(mem_usage)
    
    return timestamps, cpu_usages, mem_usages

def plot_data(timestamps, cpu_usages, mem_usages):
    plt.figure(figsize=(10, 5))
    
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, cpu_usages, label='CPU Usage (%)', color='blue')
    plt.title('CPU and Memory Usage Over Time')
    plt.ylabel('CPU Usage (%)')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(timestamps, mem_usages, label='Memory Usage (%)', color='red')
    plt.xlabel('Seconds since start (s)')
    plt.ylabel('Memory Usage (%)')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    log_file_path = '/home/fantasyyeah/Semesterwork/memory.log'
    timestamps, cpu_usages, mem_usages = parse_log_file(log_file_path)
    print(f"CPU: \n\tavg:{sum(cpu_usages)/len(cpu_usages)}\n\tmax:{max(cpu_usages)}\n\tstd:{sum([(x - sum(cpu_usages)/len(cpu_usages)) ** 2 for x in cpu_usages]) / len(cpu_usages)}")
    print(f"Memory: \n\tavg:{sum(mem_usages)/len(mem_usages)}\n\tmax:{max(mem_usages)}\n\tstd:{sum([(x - sum(mem_usages)/len(mem_usages)) ** 2 for x in mem_usages]) / len(mem_usages)}")
    plot_data(timestamps, cpu_usages, mem_usages)
