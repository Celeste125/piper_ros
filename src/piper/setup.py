from setuptools import find_packages, setup
import glob
import sys
import os
from glob import glob

package_name = 'piper'

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_single_ctrl = piper.piper_ctrl_single_node:main',
            'piper_ms_ctrl = piper.piper_start_ms_node:main',
            'piper_read_master = piper.piper_read_master_node:main',
            'piper_cartesian_controller = piper.piper_cartesian_controller:main',
            'send_cartesian_command = piper.send_cartesian_command:main',
            'piper_joint_controller_sim = piper.piper_joint_controller_sim:main',
            'piper_read_slave_joint = piper.piper_read_slave_joint:main',
            'piper_teleop_joint = piper.piper_teleop_joint:main',
            'control_manager = piper.control_manager:main',
            'delta_manager = piper.delta_manager:main',
            'joystick_controller = piper.joystick_controller:main',
            'piper_conrol_gamesir = piper.piper_conrol_gamesir:main',
            
        ],
    },
)
