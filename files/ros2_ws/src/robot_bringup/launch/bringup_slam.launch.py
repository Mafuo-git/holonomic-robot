# ===============================
# FICHIER : bringup_slam.launch.py
# ROS 2 Jazzy - VERSION CORRIGÉE COMPLÈTE
# ===============================

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import LifecycleNode
import os

def generate_launch_description():
    ld = LaunchDescription()


    robot_bringup_dir = get_package_share_directory("robot_bringup")
    urdf_file = os.path.join(robot_bringup_dir, 'URDF', 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 30.0,
            'use_tf_static': True,
        }]
    ))


    # ==================== FRONT LIDAR ====================
    
    if os.path.exists("/dev/ttyUSB0"):
        ld.add_action(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_front',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'LiDAR_AvG_Link',
                'inverted': False,
                'angle_compensate': False,
                'scan_mode': 'Sensitivity'
            }],
            remappings=[('scan', '/scan_front')],
            output='screen'
        ))
        
        ld.add_action(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    robot_bringup_dir,
                    "params", "scan_filter_front.yaml",
                ])],
            remappings=[
                ('scan', '/scan_front'),
                ('scan_filtered', '/scan_front_filtered')
            ],
        ))
    
    # ==================== REAR LIDAR ====================
    
    if os.path.exists("/dev/ttyUSB1"):
        ld.add_action(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_rear',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 256000,
                'frame_id': 'LiDAR_ArD_Link',
                'inverted': False,
                'angle_compensate': False,
                'scan_mode': 'Sensitivity'
            }],
            remappings=[('scan', '/scan_rear')],
            output='screen'
        ))
        
        ld.add_action(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    robot_bringup_dir,
                    "params", "scan_filter_rear.yaml",
                ])],
            remappings=[
                ('scan', '/scan_rear'),
                ('scan_filtered', '/scan_rear_filtered')
            ],
        ))
    
    # ==================== LASER MERGER ====================
    
    if os.path.exists("/dev/ttyUSB0") or os.path.exists("/dev/ttyUSB1"):
        ld.add_action(Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
            parameters=[PathJoinSubstitution([
                robot_bringup_dir,
                "params", "dual_laser_merger.yaml",
            ])],
            remappings=[
                ('laser_1', '/scan_front_filtered'),
                ('laser_2', '/scan_rear_filtered'),
                ('merged', '/scan')
            ],
        ))
    
    # ==================== STATIC TRANSFORMS ====================
    """
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.30', '0.20', '0.15', '0', '0', '0', 'base_link', 'LiDAR_AvG_Link']
    ))
    
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.30', '-0.20', '0.15', '0', '0', '3.14159', 'base_link', 'LiDAR_ArD_Link']
    ))"""
    
    # ==================== SLAM TOOLBOX ====================
    """
    ld.add_action(LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            PathJoinSubstitution([
                robot_bringup_dir,
                "params",
                "slam_toolbox.yaml",
            ])
        ],
        output='screen'
    ))
    """
    """
    # ==================== NAV2: MAP SERVER ====================
    
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': '',  # À remplir après cartographie
            'use_sim_time': False
        }],
        output='screen'
    ))
    
    # ==================== NAV2: AMCL ====================
    
    ld.add_action(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            PathJoinSubstitution([
                robot_bringup_dir,
                'params',
                'nav2_params_holonomic.yaml'
            ])
        ],
        output='screen'
    ))
    
    # ==================== NAV2: PLANNER SERVER ====================
    
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                robot_bringup_dir,
                'params',
                'nav2_params_holonomic.yaml'
            ])
        ]
    ))
    
    # ==================== NAV2: CONTROLLER SERVER ====================
    
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                robot_bringup_dir,
                'params',
                'nav2_params_holonomic.yaml'
            ])
        ]
    ))
    
    # ==================== NAV2: BEHAVIOR SERVER ====================
    
    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                robot_bringup_dir,
                'params',
                'nav2_params_holonomic.yaml'
            ])
        ]
    ))
    
    # ==================== NAV2: BT NAVIGATOR ====================
    
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                robot_bringup_dir,
                'params',
                'nav2_params_holonomic.yaml'
            ])
        ]
    ))
    """

    # ==================== LIFECYCLE MANAGERS ====================
    """ # Lifecycle manager pour SLAM
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    ))"""
    
    """# Lifecycle manager pour navigation
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
                'amcl'
            ]
        }]
    ))
    """
    """# Lifecycle manager pour localization & map
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    ))"""
    

    # temporary fake odom for testing
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    ))

    # ==================== RVIZ ====================
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'rviz',
        'rplidar_ros.rviz'
    )
    
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    ))

    return ld
