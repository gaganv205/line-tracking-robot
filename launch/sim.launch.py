import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('line_follower_robot')

    # Process URDF
    xacro_file  = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_desc  = xacro.process_file(xacro_file).toxml()
    world_file  = os.path.join(pkg, 'worlds', 'line_world.world')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Spawn robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'line_follower',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        output='screen'
    )

    # Line follower node
    follower = Node(
        package='line_follower_robot',
        executable='line_follower_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([gazebo, rsp, spawn, follower])
