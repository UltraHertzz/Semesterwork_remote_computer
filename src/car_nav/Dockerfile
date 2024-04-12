ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base-jammy
ENV ROS_WS=/root/ros_ws
WORKDIR $ROS_WS
COPY ./ src/car_nav 
RUN bash -c "apt update && \
    apt install -y \
        ros-humble-nav2-bringup \
        ros-humble-navigation2 && \
        source /opt/ros/humble/setup.bash && \
        cd $ROS_WS && colcon build"
CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          ros2 launch car_nav bringup_launch.py params:=${ROS_WS}/src/car_nav/params/car_nav2.yaml" ]