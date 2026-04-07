#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # RViz 配置
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='/home/jackie/robobus_localization/fusion_localization_ws/src/fusion_localization/rviz/default.rviz',
        description='Path to an RViz config file (.rviz). Leave empty to use default RViz layout.'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # rosbag 路径
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/jackie/slam_gaoxiang/dataset/2025-09-17-10-00-55',
        description='Path to the rosbag (folder). Leave empty to skip playback.'
    )
    bag_path = LaunchConfiguration('bag_path')

    fusion_node = Node(
        package='fusion_localization',  
        executable='fusion_localization_node',    
        name='fusion_localization_node',
        output='screen',
    )
    
    #IMU 预测节点（独立进程）
    imu_predictor_node = Node(
        package='fusion_localization',
        executable='imu_predictor_node',
        name='imu_predictor_node',
        output='screen',
    )
    
    #点云管理节点
    tile_manager_node = Node(
        package='fusion_localization',
        executable='tile_manager_node',
        name='tile_manager_node',
        output='screen',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config,],
    )

    # rosbag2 回放节点（若未提供 bag_path，可通过 launch 参数留空以跳过）
    rosbag_play = ExecuteProcess(
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
        rviz_config_arg,
        bag_path_arg,
        fusion_node,
        imu_predictor_node,
        tile_manager_node,
        rviz_node,
        rosbag_play,
    ])