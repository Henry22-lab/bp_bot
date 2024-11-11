from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bp_control',
            executable='robot_control_node',
            name='robot_control_node',
            output='screen'
        ),
        Node(
            package='bp_control',
            executable='wheel_tf_node',
            name='wheel_tf_node',
            output='screen'
        ),
        Node(
            package='bp_control',
            executable='odom_node',
            name='odom_node',
            output='screen'
        )
    ])

