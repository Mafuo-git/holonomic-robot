# ===============================
# bringup_slam_nav2.launch.py
# Ajout de Nav2 au bringup_slam.launch.py
# ROS 2 Jazzy
# ===============================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

   # ---------- Nav2 ----------
    nav2_params = PathJoinSubstitution([
        get_package_share_directory('robot_bringup'),
        'params',
        'nav2_params_holonomic.yaml'
    ])

    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params]
    ))

    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_params]
    ))

    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_params]
    ))

    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    ))

    return ld
