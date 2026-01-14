from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    return LaunchDescription([

        ComposableNodeContainer(
            name='phidgets_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',

            composable_node_descriptions=[

                # Phidget MotorControl 487541
                ComposableNode(
                    package='phidgets_motors',
                    plugin='phidgets::MotorsRosI',
                    name='phidgets_487541',
                    namespace='phidget_487541',
                    parameters=[{
                        'serial': 487541,
                    }],
                ),

                # Phidget MotorControl 487736
                ComposableNode(
                    package='phidgets_motors',
                    plugin='phidgets::MotorsRosI',
                    name='phidgets_487736',
                    namespace='phidget_487736',
                    parameters=[{
                        'serial': 487736,
                    }],
                ),
            ],
        ),


        # ===============================
        # Listener low-level control
        # ===============================
        Node(
            package='cpp_motors_control',
            executable='listener',
            name='listener',
            output='screen',
        ),

        # ===============================
        # Node-RED teleop
        # ===============================
        Node(
            package='cpp_motors_control',
            executable='node_red_teleop',
            name='node_red_teleop',
            output='screen',
        ),
    ])
