import matplotlib.pyplot as plt
import numpy as np

def extract_data_from_log(log_file_path):
    data_values = []
    with open(log_file_path, 'r') as file:
        for line in file:
            if 'data:' in line:
                # 提取 "data:" 后的数字并转换为浮点数
                parts = line.split('data:')
                if len(parts) > 1:
                    try:
                        data_value = float(parts[1].strip())
                        data_values.append(data_value)
                    except ValueError:
                        print(f"Warning: Unable to convert {parts[1].strip()} to float.")
    return data_values

def plot_data(data_values_container, data_values_bare_metal):
    plt.figure(figsize=(10, 5))  # 设置图形大小
    len_diff = len(data_values_container) - len(data_values_bare_metal)
    data_values_bare_metal = [np.nan]*len_diff + data_values_bare_metal

    plt.plot(data_values_container, linestyle='-', color='blue', label='Container')
    plt.plot(data_values_bare_metal, linestyle='-', color='red', label='Bare Metal')
    plt.title('Execution Time Comparison')
    plt.xlabel('Sample Number')
    plt.ylabel('Time (s)')
    plt.grid(True)
    plt.legend()
    plt.show()
if __name__ == '__main__':
    log_file_path_container = '/home/fantasyyeah/ros2_ws/log/runtime/container/topic/_exec_time.log'  # 设置日志文件的路径
    data_values_container = extract_data_from_log(log_file_path_container)
    log_file_path_bare_metal = '/home/fantasyyeah/ros2_ws/log/runtime/bare_metal/topic/_exec_time_controller_to_driver.log' 
    data_values_bare_metal = extract_data_from_log(log_file_path_bare_metal)
    avg_container = sum(data_values_container) / len(data_values_container)
    min_container = min(data_values_container)
    max_container = max(data_values_container)
    std_container = sum([(x - avg_container) ** 2 for x in data_values_container]) / len(data_values_container)
    avg_bare_metal = sum(data_values_bare_metal) / len(data_values_bare_metal)
    min_bare_metal = min(data_values_bare_metal)
    max_bare_metal = max(data_values_bare_metal)
    std_bare_metal = sum([(x - avg_bare_metal) ** 2 for x in data_values_bare_metal]) / len(data_values_bare_metal)
    print("Resbonce Time Comparison:\n\tContainer:%.4f s\n\t Bare Metal: %.4f s" %(1714658042.326516204-1714658039.429470730, 1714659610.727335780-1714659604.074638577))
    print(f"Container: \n\tnum_samples:{len(data_values_container)}\n\tavg: {avg_container} \n\tmin: {min_container} \n\tmax: {max_container}\n\tstd: {std_container}\n Bare Metal: \n\tnum_samples:{len(data_values_bare_metal)}\n\tavg: {avg_bare_metal}\n\tmin: {min_bare_metal}\n\tmax: {max_bare_metal}\n\tstd: {std_bare_metal}")
    plot_data(data_values_container, data_values_bare_metal)
 