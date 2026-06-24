import os
from glob import glob
from setuptools import setup

package_name = 'rc2026_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/map', glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hgzq',
    maintainer_email='zhuoxuanliu439@gmail.com',
    description='RC2026 Gazebo simulation package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simple_teleop = rc2026_sim.simple_teleop:main',
            'mock_wheel_odom = rc2026_sim.mock_wheel_odom:main',
            'odom_drift_injector = rc2026_sim.odom_drift_injector:main',
            'sim_relocalizer = rc2026_sim.sim_relocalizer:main',
        ],
    },
)
