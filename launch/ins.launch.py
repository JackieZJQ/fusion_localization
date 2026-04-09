#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    beidou_stream_node = Node(
        package='beidou_ins_driver',
        executable='beidou_stream_node',
        name='beidou_stream_node',
        output='screen',
    )

    ins_localization_node = Node(
        package='localization',
        executable='ins_localization_node',
        name='ins_localization_node',
        output='screen',
        # parameters=[],  # 若有参数 YAML，可在此添加
        # remappings=[('/beidou/inspva', '/your/inspva/topic'),
        #             ('/beidou/corrimudata', '/your/imu/topic')],
    )

    return LaunchDescription([
        beidou_stream_node,
        ins_localization_node,
    ])