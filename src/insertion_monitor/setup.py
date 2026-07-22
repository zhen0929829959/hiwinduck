from setuptools import find_packages, setup


package_name = 'insertion_monitor'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        exclude=['test']
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools'
    ],
    zip_safe=True,
    maintainer='annie',
    maintainer_email='annie@example.com',
    description='RJ45 insertion state monitor',
    license='Apache-2.0',
    tests_require=[
        'pytest'
    ],
    entry_points={
        'console_scripts': [
            (
                'insertion_monitor_node = '
                'insertion_monitor.'
                'insertion_monitor_node:main'
            ),
        ],
    },
)