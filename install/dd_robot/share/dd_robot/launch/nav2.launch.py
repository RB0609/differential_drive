import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # --- 1. Setup Path Variables ---
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # --- 2. Declare Launch Arguments (To make the script reusable) ---
    # Arguments from your command line:
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/rakesh/map.yaml',
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/home/rakesh/ros2_ws/src/differential_drive/dd_robot/config/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # --- 3. Include Nav2 Bringup Launch ---
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
        ),
        # Pass the launch arguments defined above into the included file
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items(),
    )

    # --- 4. Include Rviz Launch ---
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'rviz_launch.py'])
        ),
        # Ensure Rviz also uses simulation time
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # --- 5. Return the Launch Description ---
    return LaunchDescription([
        # Declare all arguments first
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        
        # Execute the two launch files
        bringup_cmd,
        rviz_cmd,
    ])