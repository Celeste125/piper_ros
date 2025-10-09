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

    # Define launch parameters
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port for the robot arm'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Enable robot arm automatically'
    )

    # Include display_xacro.launch.py DESHABILITANDO el GUI
    display_xacro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(piper_description_path),
        launch_arguments={
            'gui': 'false',  # ← PARÁMETRO CORRECTO
            'use_sim_time': 'true',
            'use_joint_state_publisher':'false'
        }.items()
    )

    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Gripper existence flag'
    )
    
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='2',
        description='gripper'
    )

    # Nodo de control original por joints
    piper_ctrl_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        parameters=[
            {'can_port': LaunchConfiguration('can_port')},
            {'auto_enable': LaunchConfiguration('auto_enable')},
            {'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple')},
            {'gripper_exist': LaunchConfiguration('gripper_exist')}
        ],
        remappings=[
            ('joint_ctrl_single', '/joint_states')
        ]
    )

    # NUEVO: Controlador Cartesian
    cartesian_controller_node = Node(
        package='piper',
        executable='piper_cartesian_controller',
        name='piper_cartesian_controller',
        output='screen',
        parameters=[]
    )

    keyboard_node = Node(
        package='piper',
        executable='piper_keyboard_control',
        name='piper_keyboard_control',
        output='screen'
    )

    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        display_xacro_launch,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        piper_ctrl_node,
        #cartesian_controller_node,
        #keyboard_node
    ])