import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
import xacro


def generate_launch_description():
    
    # 1. DECLARE LAUNCH ARGUMENT FOR use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Package Directories
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    urdf_path = os.path.join(pkg_description, 'urdf', 'my_robot2.urdf.xacro')
    gazebo_config_path = os.path.join(pkg_bringup, 'config', 'gazebo_bridge.yaml')
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')
    world_path = os.path.join(pkg_bringup, 'worlds', 'test_world.sdf')
    # world_path = os.path.join(pkg_bringup, 'worlds', 'lidar_world.sdf')

   

    # Process the URDF
    robot_description_config = xacro.process_file(urdf_path, mappings={'robot_name':"robot1"})
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # 2. ADD use_sim_time TO ROBOT STATE PUBLISHER NODE
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description, 
            {'use_sim_time': use_sim_time}
        ] 

    )
    
    # Gazebo Sim (Headless Mode)
    # Note: gz_sim.launch.py internally handles the clock and use_sim_time for the server
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
            # 'gz_args': f'-r empty.sdf'
        }.items()
    )
    
    # 3. ADD use_sim_time TO SPAWN ROBOT NODE
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # <-- ADDED
    )
    
    # ROS-Gazebo Bridge (The bridge itself typically doesn't need use_sim_time unless it's a dedicated clock bridge, 
    # but it's good practice for nodes interacting with the simulation)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': gazebo_config_path},
            {'use_sim_time': use_sim_time} # <-- ADDED (Good practice)
        ],
        output='screen'
    )
    
    # 4. ADD use_sim_time TO RVIZ2 NODE
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # <-- ADDED
    )

    static_tf_fix_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_fixer',
        output='screen',
        arguments=[
            '0', '0', '0',  # x, y, z (translation is 0 since they are the same point)
            '0', '0', '0',  # roll, pitch, yaw (rotation is 0)
            'lidar_link',   # PARENT Frame ID (the correct link in your URDF)
            'my_robot/base_footprint/gpu_lidar' # CHILD Frame ID (the broken name in your /scan echo)
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd, # <-- Declare the argument first
        robot_state_publisher_node,
        gazebo_sim,
        static_tf_fix_node,
        spawn_robot_node,
        bridge_node,
        # rviz_node
    ])