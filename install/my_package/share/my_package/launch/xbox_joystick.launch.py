from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),
        Node(
            package='my_package',     
            executable='turtle_joystick.py', 
            output='screen'
        ),
    ])

