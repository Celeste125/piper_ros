from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    joystick_controller_node = Node(
        package='piper_teleoperation',
        executable='joystick_controller_node.py',
        name='joystick_controller_node',
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    
    return LaunchDescription([
        joy_node,
        joystick_controller_node,
    ])