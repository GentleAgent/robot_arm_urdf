import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Include the Gazebo empty world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'empty_world.launch.py'
                ])
            )
        ),

        # Static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
        ),

        # Spawn the URDF model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=[
                '-file', PathJoinSubstitution([
                    FindPackageShare('robot_arm_urdf'),
                    'urdf',
                    'robot_arm_urdf.urdf'
                ]),
                '-entity', 'robot_arm_urdf'
            ],
            output='screen'
        ),

        # Publish a fake joint calibration message
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' topic pub /calibrated std_msgs/msg/Bool "{data: true}" --once'
            ]],
            shell=True
        )
    ])

