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
    piper_description_path = os.path.join(
        get_package_share_directory('piper_description'),
        'launch',
        'piper_with_gripper',
        'display_xacro.launch.py'
    )

    # Include display_xacro.launch.py 
    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path),
        launch_arguments={
            'gui': 'false',
            'use_sim_time': 'false'
        }.items()
    )

    # NUEVO: Controlador de joints para simulación
    joint_controller_node = Node(
        package='piper',
        executable='piper_joint_controller_sim',
        name='piper_joint_controller_sim',
        output='screen',
        parameters=[]
    )

    # Controlador Cartesian
    cartesian_controller_node = Node(
        package='piper',
        executable='piper_cartesian_controller',
        name='piper_cartesian_controller',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        display_xacro_launch,
        joint_controller_node,  # NUEVO - Controlador de joints para simulación
        cartesian_controller_node
    ])