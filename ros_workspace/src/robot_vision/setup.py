from setuptools import setup

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'opencv-contrib-python'
    ],
    zip_safe=True,
    maintainer='tsimafei',
    maintainer_email='tsimafei.liashkevich@telecom-sudparis.eu',
    description='robot_vision node aims to provide distance measurement and aruco tag scanning possibilities '
                'to the robot',
    author='tsimafei',
    author_email='tsimafei.liashkevich@telecom-sudparis.eu',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
