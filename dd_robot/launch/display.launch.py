import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    dd_robot = get_package_share_directory("dd_robot")
    bringup_dir = get_package_share_directory('nav2_bringup')
   
    urdf_path = os.path.join(dd_robot,'urdf','robot.urdf.xacro')
    bridge_yaml = os.path.join(dd_robot,'config','gazebo_bridge.yaml')
    nav2_yaml = os.path.join(dd_robot, 'config', 'ekf.yaml')
    rviz_path = os.path.join(dd_robot,'rviz','robot.rviz')
    nav2_path= os.path.join(dd_robot,'config', 'nav2_params.yaml')
   
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(dd_robot).parent.resolve())
            ]
        )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf"]
                    )
                ]
             )
    
    #nav2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(bringup_dir, 'launch' ,'bringup_launch.py')]),
                                           #launch_arguments=[{'params_file': nav2_path, 'use_sim_time': 'true'}.items()])
    

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "dd_robot"],
    )
 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description},{'use_sim_time':use_sim_time}],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d",rviz_path],
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}]
    )
   

    



    
    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
    )
    

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    name="joint_state_broadcaster_spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    output="screen"
    )

# Spawner for diff_drive_controller
    dd_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    name="dd_controller_spawner",
    arguments=["dd_controller", "--controller-manager", "/controller_manager"],
    output="screen"
    )
    


 

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )
    
   

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo,
        gazebo_resource_path,
        #robot_localization_node,
        rviz2,
       
      
    
        gz_spawn_entity,
        bridge_node,
        #joint_state_broadcaster_spawner,
        #dd_controller_spawner,
        #nav2_launch
        
        
      
    ])
