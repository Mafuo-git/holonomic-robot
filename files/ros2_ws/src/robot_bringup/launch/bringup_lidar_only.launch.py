from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()


    rviz_config_dir = os.path.join(
            get_package_share_directory('rplidar_ros'),
            'rviz',
            'rplidar_ros.rviz')

    ## Front Lidar
    if os.path.exists("/dev/ttyUSB0"):
        ld.add_action(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_front',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'laser_front',
                'inverted': False,
                'angle_compensate': False,
                'scan_mode': 'Sensitivity'
            }],
            remappings=[('scan', '/scan_front')],
            output='screen'

        ))

        """ld.add_action(Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_front',
            remappings=[
                ('scan', '/scan_front'),
                ('scan_filtered', '/scan_front_filtered')
            ],
            parameters=[
                os.path.join(
                    get_package_share_directory("robot_bringup"),
                    "params", "scan_filter_front.yaml",
                )],
        ))"""

        ld.add_action(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_bringup"),
                    "params", "scan_filter_front.yaml",
                ])],
            remappings=[
                ('scan', '/scan_front'),
                ('scan_filtered', '/scan_front_filtered')
            ],
        ))


    ## Rear Lidar
    if os.path.exists("/dev/ttyUSB1"):
        ld.add_action(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_rear',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 256000,
                'frame_id': 'laser_rear',
                'inverted': False,
                'angle_compensate': False,
                'scan_mode': 'Sensitivity'
            }],
            remappings=[('scan', '/scan_rear')],
            output='screen'
        ))

        """ld.add_action(Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_rear',
            remappings=[
                ('scan', '/scan_rear'),
                ('scan_filtered', '/scan_rear_filtered')
            ],
            parameters=[
                os.path.join(
                    get_package_share_directory("robot_bringup"),
                    "params", "scan_filter_rear.yaml",
                )],
        ))"""

        ld.add_action(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_bringup"),
                    "params", "scan_filter_rear.yaml",
                ])],
            remappings=[
                ('scan', '/scan_rear'),
                ('scan_filtered', '/scan_rear_filtered')
            ],
        ))

    ## Merging scans if both lidars are present
    if os.path.exists("/dev/ttyUSB0") and os.path.exists("/dev/ttyUSB1"):
        ld.add_action(Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
            remappings=[
                ('laser_1', '/scan_front_filtered'),
                ('laser_2', '/scan_rear_filtered'),
                ('merged', '/scan_merged')
            ],
            parameters=[os.path.join(
                get_package_share_directory('robot_bringup'),
                'params',
                'dual_laser_merger.yaml'
            )]
        ))

        
    
    ## Static transforms
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.30', '0.20', '0.15', '0', '0', '0', 'base_link', 'laser_front']
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.30', '-0.20', '0.15', '0', '0', '3.14159', 'base_link', 'laser_rear']
    ))

    

    ## RViz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    ))
    
    
    return ld
