from setuptools import find_packages, setup

package_name = 'ur_pilz_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur_moveit_pilz.launch.py']),
        ('share/' + package_name + '/config', ['config/pilz_cartesian_limits.yaml', 'config/pilz_industrial_motion_planner_planning_planner.yaml']),
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
        ],
    },
)
