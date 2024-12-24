from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dsr_task',
            executable='gripper_control',
            output='screen',
            name='gripper_control',
        ),
        Node(
            package='dsr_task',
            executable='homming',
            output='screen',
            name='homming',
        ),
        Node(
            package='dsr_task',
            executable='tool_changer',
            output='screen',
            name='tool_changer',
        ),
    ])
