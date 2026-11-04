from glob import glob
from setuptools import find_packages, setup

package_name = 'ar_lane_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.todo',
    description='Camera lane perception: publishes ar_msgs/LaneError on /perception/lane_error',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_perception_node = ar_lane_perception.camera_perception_node:main',
        ],
    },
)
