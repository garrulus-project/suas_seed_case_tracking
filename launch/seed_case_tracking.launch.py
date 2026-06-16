#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('suas_seed_case_tracking'),
        'config'
    )
    default_config = os.path.join(config_dir, 'basler_ace_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Parameter YAML file for seed_case_tracking_node'),
        DeclareLaunchArgument('mode', default_value='data_collection',
                              description='Operating mode: data_collection or online_tracking'),
        DeclareLaunchArgument('camera_type', default_value='basler',
                              description='Camera type: basler or dummy'),
        DeclareLaunchArgument('gps_topic', default_value='/mavros/global_position/global',
                              description='GPS topic'),
        DeclareLaunchArgument('imu_topic', default_value='/mavros/imu/data',
                              description='IMU topic'),
        DeclareLaunchArgument('cougar_image_topic', default_value='/cougar_camera/camera_driver_node/image_raw',
                              description='Cougar image topic'),
        DeclareLaunchArgument('sprouter_status_topic', default_value='/mavros/sprouter/status', # When the sprouter propells the seed
                              description='Sprouter status topic'),
        DeclareLaunchArgument('sprouter_released_topic', default_value='/mavros/sprouter/seed_released', # Current status of the sprouter
                              description='Sprouter seed released topic'),
        DeclareLaunchArgument('sprouter_debug_value_topic', default_value='/mavros/debug_value/named_value_int',
                              description='Sprouter debug value topic'),
        Node(
            package='suas_seed_case_tracking',
            executable='seed_case_tracking_node',
            name='seed_case_tracking_node',
            namespace='seed_case_tracking',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'mode': LaunchConfiguration('mode'),
                    'camera_type': LaunchConfiguration('camera_type'),
                    'gps_topic': LaunchConfiguration('gps_topic'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'cougar_image_topic': LaunchConfiguration('cougar_image_topic'),
                    'sprouter_status_topic': LaunchConfiguration('sprouter_status_topic'),
                    'sprouter_released_topic': LaunchConfiguration('sprouter_released_topic'),
                    'sprouter_debug_value_topic': LaunchConfiguration('sprouter_debug_value_topic'),
                },
            ],
        ),
    ])
