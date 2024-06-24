import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_arm_urdf'),
                'world',
                'pick_drop1.world'
            ]),
            description='Path to the world file'
        ),
        DeclareLaunchArgument('arg_x', default_value='0.00', description='X coordinate'),
        DeclareLaunchArgument('arg_y', default_value='0.00', description='Y coordinate'),
        DeclareLaunchArgument('arg_z', default_value='0.00', description='Z coordinate'),
        DeclareLaunchArgument('arg_R', default_value='0.00', description='Roll'),
        DeclareLaunchArgument('arg_P', default_value='0.00', description='Pitch'),
        DeclareLaunchArgument('arg_Y', default_value='0.00', description='Yaw'),

        # Include the Gazebo empty world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ),
            launch_arguments={'world': LaunchConfiguration('world')}.items()
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
            name='spawn_urdf',
            arguments=[
                '-x', LaunchConfiguration('arg_x'),
                '-y', LaunchConfiguration('arg_y'),
                '-z', LaunchConfiguration('arg_z'),
                '-Y', LaunchConfiguration('arg_Y'),
                '-param', 'robot_description',
                '-urdf',
                '-model', 'robot_arm_urdf',
                '-J', 'joint_1', '0.0',
                '-J', 'joint_2', '0.0',
                '-J', 'joint_3', '0.0',
                '-J', 'joint_4', '0.0',
                '-J', 'joint_5', '0.0',
                '-J', 'joint_6', '0.0',
                '-J', 'joint_7', '0.0'
            ],
            output='screen'
        ),

        # Load and launch the joint trajectory controller
        ExecuteProcess(
            cmd=[
                'ros2', 'param', 'load', 'controller_manager',
                PathJoinSubstitution([
                    FindPackageShare('robot_arm_urdf'),
                    'config',
                    'joint_trajectory_controller.yaml'
                ])
            ],
            shell=True
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            arguments=['joint_state_controller', 'robot_arm_controller', 'hand_ee_controller'],
            output='screen'
        ),

        # Robot State Publisher for TF of each joint
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    PathJoinSubstitution([
                        FindPackageShare('robot_arm_urdf'),
                        'urdf',
                        'robot_arm_urdf.urdf'
                    ]),
                    value_type=str
                )
            }]
        )
    ])

