from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzz',
    maintainer_email='zzz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'strategy_example = ur_robot_control.strategy_example:main',
            'hand_in_eye_calibration = ur_robot_control.hand_in_eye_calibration:main',
            'three_points_calibration_example = ur_robot_control.three_points_calibration_example:main',
            'check_right = ur_robot_control.check_right:main',
            'camera_flange_matrix2 = ur_robot_control.camera_flange_matrix2:main',
            'hiwin_pose_publisher = ur_robot_control.hiwin_pose_publisher:main',
            'hiwin_tf_tree = ur_robot_control.hiwin_tf_tree_publisher:main',
            'strategy_example_3 = ur_robot_control.strategy_example_3:main',
            'strategy_example_mul = ur_robot_control.strategy_example_mul:main',
            'force=ur_robot_control.force:main',
            'continuous_insertion_test=ur_robot_control.continuous_insertion_test:main'
        ],
    },
)
