# ===============================
# FICHIER : bringup_slam.launch.py
# ROS 2 Jazzy
#  - 2 RPLIDAR A3
#  - Fusion via ira_laser_tools
#  - slam_toolbox (sans odom)
# ===============================

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ---------- RPLIDAR AVANT ----------
    rplidar_front = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_front',
        parameters=[{
            'serial_port': '/dev/rplidar_front',
            'serial_baudrate': 256000,
            'frame_id': 'laser_front',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'angle_min': -2.35,
            'angle_max':  2.35
        }],
        output='screen'
    )

    # ---------- RPLIDAR ARRIÈRE ----------
    rplidar_rear = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_rear',
        parameters=[{
            'serial_port': '/dev/rplidar_rear',
            'serial_baudrate': 256000,
            'frame_id': 'laser_rear',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'angle_min': -2.35,
            'angle_max':  2.35
        }],
        remappings=[('scan', 'scan_rear')],
        output='screen'
    )

    # ---------- TF STATIQUES ----------
    tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.30', '0.20', '0.15', '0', '0', '0', 'base_link', 'laser_front']
    )

    tf_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.30', '-0.20', '0.15', '0', '0', '3.14159', 'base_link', 'laser_rear']
    )

    # ---------- FUSION DES SCANS ----------
    laser_merger = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_merger',
        parameters=[{
            'destination_frame': 'base_link',
            'scan_destination_topic': 'scan_merged',
            'cloud_destination_topic': 'cloud_merged',
            'laserscan_topics': ['scan', 'scan_rear'],
            'angle_min': -3.14,
            'angle_max':  3.14,
            'angle_increment': 0.0058,
            'range_min': 0.15,
            'range_max': 25.0
        }],
        output='screen'
    )

    # ---------- SLAM TOOLBOX ----------
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_odom': False,
            'use_scan_matching': True,
            'scan_topic': 'scan_merged',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05
        }],
        output='screen'
    )

    return LaunchDescription([
        rplidar_front,
        rplidar_rear,
        tf_front,
        tf_rear,
        laser_merger,
        slam_toolbox
    ])

