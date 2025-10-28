import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_description, 'urdf', 'my_robot2.urdf.xacro')
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'lidar_camera_config.rviz')
    world_path = os.path.join(pkg_bringup, 'worlds', 'test_world.sdf')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

    robot_positions = [
        ('robot1', 0.0, 0.0, 0.0),
        ['robot2', 2.0, 0.0, 0.0],
        ['robot3', 0.0, 2.0, 0.0],
    ]

    robot_groups = []

    for name, x, y, z in robot_positions:
        robot_description_config = xacro.process_file(
            urdf_path,
            mappings={'robot_name': name}
        )
        robot_description = {'robot_description': robot_description_config.toxml()}
        

        group = GroupAction(
            [
                PushRosNamespace(name),

 
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name=f'{name}_state_pub',
                    output='screen',
                    parameters=[robot_description, {'use_sim_time': use_sim_time}],
                   
                    # remappings=[
                    #     ('/clock', '/clock'),
                    #     ('/joint_states', 'joint_states')
                    # ]
                ),
                
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'spawn_{name}',
                    arguments=[
                        '-name', name,
                        '-topic', 'robot_description',
                        '-x', str(x), '-y', str(y), '-z', str(z)

                    ]
                ),

                Node(
                    package= 'ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'{name}_bridge',
                    arguments=[

                        f'/world/empty/model/{name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                        # f'/world/empty/model/{name}/joint_state@sensor_msgs/msg/JointState@/joint_states[gz.msgs.JointState',

                        f'/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

                        f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

                        f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                        f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                        
                        f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

                    ],
                    output="screen",
                    parameters=[{'use_sim_time': use_sim_time}],
                    remappings=[
                        (f'/model/{name}/tf', '/tf'),
                        (f'/world/empty/model/{name}/joint_state', f'/{name}/joint_states'),
                        ('/scan', 'scan')
                    ]
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='world_frame',
                    output='screen',
                    arguments=[
                        str(x), str(y), str(z),  # x, y, z (translation is 0 since they are the same point)
                        '0', '0', '0',  # roll, pitch, yaw (rotation is 0)
                        'odom',   # PARENT Frame ID (the correct link in your URDF)
                        f'{name}/odom' # CHILD Frame ID (the broken name in your /scan echo)
                    ],
                    parameters=[{'use_sim_time': use_sim_time}]
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='lidar_frame_fixer',
                    output='screen',
                    arguments=[
                        '0', '0', '0',  # x, y, z (translation is 0 since they are the same point)
                        '0', '0', '0',  # roll, pitch, yaw (rotation is 0)
                        f'{name}/lidar_link',   # PARENT Frame ID (the correct link in your URDF)
                        f'{name}/{name}/base_footprint/gpu_lidar' # CHILD Frame ID (the broken name in your /scan echo)
                    ],
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        )
        robot_groups.append(group)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame',
        output='screen',
        arguments=[
            '0', '0', '0',  # x, y, z (translation is 0 since they are the same point)
            '0', '0', '0',  # roll, pitch, yaw (rotation is 0)
            'world',   # PARENT Frame ID (the correct link in your URDF)
            'odom' # CHILD Frame ID (the broken name in your /scan echo)
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            gazebo_sim,
            clock_bridge,
            static_tf_world,
            rviz_node
            # static_tf_fix_node

        ] + robot_groups
    )