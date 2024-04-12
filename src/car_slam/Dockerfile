FROM ros:humble-ros-base-jammy as builder
ENV ROS_WS=/root/ros_ws
#WORKDIR /home/orangepi/ros2_ws
WORKDIR $ROS_WS

COPY ./ src/car_slam

RUN bash -c "apt update && \
    apt-get install -y ros-humble-slam-toolbox && \
    source /opt/ros/humble/setup.bash && \
    cd $ROS_WS && colcon build"
    
CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          ros2 launch car_slam online_async_launch params:=${ROS_WS}/src/car_slam/params/online_async_mapping.yaml" ]
