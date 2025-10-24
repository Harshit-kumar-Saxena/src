from setuptools import setup
from glob import glob
import os

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install the inner robot_description folder with URDFs
        ('share/' + package_name + '/robot_description/urdf', glob('robot_description/urdf/*.xacro')),
        ('share/' + package_name + '/robot_description/config', glob('robot_description/config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harshitji',
    maintainer_email='you@example.com',
    description='Python ROS2 package for robotic arm',
    license='Apache License 2.0',
)
