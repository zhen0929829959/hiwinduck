from setuptools import setup
import os
from glob import glob

package_name = 'hiwin_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/config', [
            'config/hiwin_tf_params.yaml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='808790017@gms.tku.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy_example = hiwin_example.strategy_example:main',
            'hand_in_eye_calibration = hiwin_example.hand_in_eye_calibration:main',
            'three_points_calibration_example = hiwin_example.three_points_calibration_example:main',
            'check_right = hiwin_example.check_right:main',
            'camera_flange_matrix2 = hiwin_example.camera_flange_matrix2:main',
            'hiwin_pose_publisher = hiwin_example.hiwin_pose_publisher:main',
            'hiwin_tf_tree = hiwin_example.hiwin_tf_tree_publisher:main',
            'strategy_example_3 = hiwin_example_mul.strategy_example_3:main',
            'strategy_example_mul = hiwin_example_mull.strategy_example:main',
            'continuous_insertion_test=hiwin_example.continuous_insertion_test:main'

        ],
    },
)
