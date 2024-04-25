import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Set ROS domain ID to 30
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')

    # config
    # slam_toolbox_param_dir = DeclareLaunchArgument('params_file', 
                                            # default_value='/home/fantasyyeah/ros2_ws/src/slam_toolbox/online_async_localization.yaml')
    slam_toolbox_param_dir = LaunchConfiguration('slam_params_file',default='/home/fantasyyeah/ros2_ws/src/slam_toolbox/online_async_localization.yaml')


    car_nav_dir = get_package_share_directory('car_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(car_nav_dir,'maps','map.yaml'))
    nav2_param_dir = LaunchConfiguration('nav_params_file',default=os.path.join(car_nav_dir,'param','car_nav2.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    # Nodes
    data_sync_py_launch = Node(
        package='sensor_sync_py',
        executable='sensor_sync_node_py',
        name='sensor_sync_py'
    )
    lidar_odom_launch = Node(
        package='lidar_odom_from_scratch',
        executable='lidar_odom_from_scratch_node',
        name='lidar_odom_node',
        output='screen'
    )
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={'params_file' : slam_toolbox_param_dir}.items(),
    )
    nav2_bringup_launch = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir,'launch','bringup_launch.py')),
        PythonLaunchDescriptionSource(os.path.join(car_nav_dir,'launch','bringup_launch.py')),
        launch_arguments={'map':map_yaml_path,'params_file':nav2_param_dir}.items(),
    )
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')

    # set woking flow
    return LaunchDescription([
        set_domain,
        #lidar_odom_launch,
        #data_sync_launch,
        #data_sync_py_launch,
        # slam_toolbox_launch,
        nav2_bringup_launch,
        rviz_node
    ])
