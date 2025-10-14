from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the piper_description package

    use_analytic_ik_arg= DeclareLaunchArgument(
        'use_analytic_ik',
        default_value='false',
        description='Use analytical IK instead of numerical'
    )
    dh_type_arg= DeclareLaunchArgument(
        'dh_type',
        default_value='modified',
        description='DH type: standard or modified'
    )


    # NUEVO: Controlador Cartesian
    piper_controller = Node(
        package='piper',
        executable='control_manager',
        name='control_manager',
        output='screen',
    )

    joint_delta_manager = Node(
        package='piper',
        executable='delta_manager',
        name='delta_manager',
        output='screen',
    )

    cartesian_delta_manager = Node(
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

    return LaunchDescription([
        dh_type_arg,
        use_analytic_ik_arg,
        piper_controller,
        joint_delta_manager,
        cartesian_delta_manager,
    ])