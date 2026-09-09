import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzz',
    maintainer_email='zhen0929829959@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_pub = yolo.camera_node_pub:main',
            'yolo_sub = yolo.camera_node_sub:main',
            'apriltag=yolo.camera_node_apriltag:main',
            'yolo_apriltag=yolo.camera_node_yolo_apriltag:main',
            'stereo_depth = yolo.stereo_depth_node:main',
            'yolo_left = yolo.yolo:main',
            'stereo_apriltag_calibrator = yolo.stereo_apriltag_calibrator:main',
            'apriltag_stereo_depth_debug = yolo.apriltag_stereo_depth_debug:main',
            'combined_vision_view = yolo.combined_vision_view_node:main',
        ],
    },
)
