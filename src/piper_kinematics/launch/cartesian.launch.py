from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_analytic_ik',
            default_value='false',
            description='Use analytical IK instead of numerical'
        ),
        DeclareLaunchArgument(
            'dh_type',
            default_value='modified',
            description='DH type: standard or modified'
        ),
        
        # Nodo de control cartesiano
        Node(
            package='piper_kinematics',
            executable='cartesian_controller',
            name='cartesian_controller',
            output='screen',
            parameters=[{
                'use_analytic_ik': LaunchConfiguration('use_analytic_ik'),
                'dh_type': LaunchConfiguration('dh_type'),
                'max_iterations': 80,
                'position_tolerance': 1e-3,
                'orientation_tolerance': 1e-2,
                'damping_factor': 0.1,
                'publish_rate': 30.0
            }]
        )
    ])