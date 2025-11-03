#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation')
    #world_file = os.path.join(pkg_share, 'resource', 'track.world')
    world_file = os.path.join(pkg_share, 'resource', 'lane_track.world')
    urdf_xacro = os.path.join(pkg_share, 'resource', 'car.xacro')

    # 官方的 gazebo.launch.py
    gz_pkg = get_package_share_directory('gazebo_ros')
    gz_launch = os.path.join(gz_pkg, 'launch', 'gazebo.launch.py')
    # —— 这一步展平 xacro —— 
    doc = xacro.process_file(urdf_xacro)
    robot_description_xml = doc.toxml()
 
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_sim_car',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_car',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )
    return LaunchDescription([
        SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'resource', 'models')),
        # 1) 启动 Gazebo 并加载 world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'world': world_file}.items(),
        ),

        # 2) 发布 robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_xml}],
        ),

        # 3) 延迟 5 秒再 spawn 模型，确保 /spawn_entity 服务已就绪
        TimerAction(
            period=3.0,
            actions=[ spawn_node ]
        ),
        TimerAction(
    period=4.0,
    actions=[
        Node(
            package='simulation',
            executable='lane_following',
            name='lane_following_node',
            output='screen',
            parameters=[{
                'max_speed': 0.8,   # 仿真里可适当快些
                'kp': 2.0,
                'kd': 0.6,
            }],
            # 如果你的相机话题不是 /front_camera/image_raw，请在这里 remap：
            # remappings=[('/front_camera/image_raw', '/camera/image_raw')],
        ),
        Node(
            package='simulation',
            executable='viz',
            name='visualization_node',
            output='screen',
        ),
    ]
),
    ])
