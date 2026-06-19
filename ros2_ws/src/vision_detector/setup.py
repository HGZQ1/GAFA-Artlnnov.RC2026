from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'vision_detector'

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
    maintainer='hgzq',
    maintainer_email='HGZQ2108299415@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'detector_node = vision_detector.detector_node:main',
            'model_switcher = vision_detector.model_switcher:main'
        ],
    },
)
