from setuptools import setup, find_packages

package_name = 'cmd_vel_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pid_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/cmd_vel_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hgzq',
    maintainer_email='HGZQ2108299415@outlook.com',
    description='Nav2 cmd_vel 到串口协议桥接',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = cmd_vel_bridge.bridge_node:main',
            'wheel_odom_publisher = cmd_vel_bridge.wheel_odom_publisher:main',
        ],
    },
)
