from setuptools import setup, find_packages

package_name = 'decision_processor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='hgzq',
    maintainer_email='HGZQ2108299415@outlook.com',
    description='底盘决策与数据处理节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor_node    = decision_processor.processor_node:main',
            'imu_processor     = decision_processor.imu_processor:main',
            'odometry_fusion   = decision_processor.odometry_fusion:main',
            'meilin_navigator  = decision_processor.meilin_navigator:main',
            'game_controller   = decision_processor.game_controller:main',
            'mock_feedback     = decision_processor.mock_feedback_node:main',
            'waypoint_nav      = decision_processor.waypoint_navigator:main',
            'fine_align_node   = decision_processor.fine_align_node:main',
            'dock_align_node   = decision_processor.dock_align_node:main',
            'ir_key_receiver   = decision_processor.ir_key_receiver:main',
        ],
    },
)

# from setuptools import find_packages, setup

# package_name = 'decision_processor'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='hgzq',
#     maintainer_email='HGZQ2108299415@outlook.com',
#     description='TODO: Package description',
#     license='Apache-2.0',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
