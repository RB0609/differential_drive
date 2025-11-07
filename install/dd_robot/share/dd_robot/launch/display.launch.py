import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dd_robot = get_package_share_directory("dd_robot")
    
    # --- Paths ---
    urdf_path = os.path.join(dd_robot, 'urdf', 'robot.urdf.xacro')
    #sdf_path= os.path.join(dd_robot,'urdf','mobile_base_gazebo.sdf')
    bridge_yaml = os.path.join(dd_robot, 'config', 'gazebo_bridge.yaml')
    nav2_yaml = os.path.join(dd_robot, 'config', 'ekf.yaml')
    rviz_path = os.path.join(dd_robot, 'rviz', 'dd_robot.rviz')
    default_world = os.path.join(dd_robot, 'worlds', 'model.sdf')
    

    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world, description="Gazebo world name"
    )
    world = LaunchConfiguration('world')

    # --- Gazebo resource search path (so gz can find your assets/worlds/models) ---
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(dd_robot).parent.resolve())]
    )

    # --- Robot Description (xacro -> URDF) ---
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    #robot_description = Path(sdf_path).read_text()


    # --- Gazebo Harmonic (ros_gz_sim) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={'gz_args': ['-v 4 -r ', world]}.items()
    )

    # --- Robot State Publisher (publishes TF from URDF) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        #parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- RViz ---
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- OPTIONAL: Robot Localization (leave commented until odom/imu wired) ---
    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
     )

    # --- Spawn the robot into Gazebo from /robot_description ---
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        
        output="screen",
      
        arguments=["-topic", "robot_description", "-name", "dd_robot", "-allow_renaming", "true"],
        #arguments=["-file", sdf_path, "-name", "dd_robot", "-allow_renaming", "true"],
      
        
    )

    # --- Controllers config path (your YAML) ---
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('dd_robot'), 'config', 'dd_control.yaml']
    )

    # --- Controller spawners (do NOT start immediately) ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    dd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dd_controller", "--param-file", robot_controllers],
        output='screen'
    )

    # --- Sequencing: spawn -> JSB -> diff drive (like the demo) ---
    after_spawn_load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    after_jsb_load_diff = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dd_controller_spawner],
        )
    )

    # --- Bridge (fix missing closing bracket) ---
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}, {'use_sim_time': True}],
        # Add any extra ad-hoc bridges you need; keeping /clock explicit here:
        #arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen'
    )

    
    
   

  

    return LaunchDescription([
        declare_gazebo_world,
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        #joint_state_broadcaster_spawner,
        robot_localization_node,
        bridge_node,
        #rviz2_node,
       

        # Spawn first
        gz_spawn_entity,

        
       
      

        # Then controllers in order
        #after_spawn_load_jsb,
        #after_jsb_load_diff,
    ])
