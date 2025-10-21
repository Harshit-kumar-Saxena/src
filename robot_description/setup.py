from setuptools import setup
import os
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Package metadata
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # URDF / Xacro files
        (os.path.join('share', package_name, 'urdf'),
        glob('robot_description/urdf/*.xacro')),

        # Config files (optional)
        (os.path.join('share', package_name, 'config'),
         glob('robot_description/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harshitji',
    maintainer_email='saxena150723@gmail.com',
    description='Robot description package with URDF/XACRO and launch files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)