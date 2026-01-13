#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 配置文件路径参数
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('fusion_localization'),
            'config',
            'localization_robosense.yaml'
        ]),
        description='Path to the configuration YAML file'
    )
    config_path = LaunchConfiguration('config_path')

    # RViz 配置
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('fusion_localization'),
            'rviz',
            'default.rviz'
        ]),
        description='Path to an RViz config file (.rviz). Leave empty to use default RViz layout.'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # rosbag 路径
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Path to the rosbag (folder). Leave empty to skip playback.'
    )
    bag_path = LaunchConfiguration('bag_path')

    # 融合定位节点
    fusion_node = Node(
        package='fusion_localization',  
        executable='fusion_localization_node',    
        name='fusion_localization_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
        }]
    )

    # RViz可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # rosbag2 回放节点（可选）
    # 使用条件：只有当提供了 bag_path 时才执行
    rosbag_play = ExecuteProcess(
        condition=lambda context: context.launch_configurations.get('bag_path', '') != '',
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            # '--clock',        # 若需要发布 /clock，可保留
            # '--rate', '1.0',  # 需要调整速率可放开
        ],
        output='screen',
        emulate_tty=True,
        name='ros2_bag_play',
    )

    return LaunchDescription([
        config_path_arg,
        rviz_config_arg,
        bag_path_arg,
        fusion_node,
        rviz_node,
        rosbag_play,
    ])