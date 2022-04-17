from setuptools import setup

package_name = 'motion_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsimafei',
    maintainer_email='tsimafei.liashkevich@telecom-sudparis.eu',
    description='ROS2 python node, which interacts with lidar and outputs the optimal trajectory using Dstar algorithm.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = motion_control.motion_control:main',
        ],
    },
)
