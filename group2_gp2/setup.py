from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'group2_gp2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxz',
    maintainer_email='maxyk@umd.edu',
    description='ROS 2 action server and client for GP2 navigate-to-goal mission using a two-phase proportional controller.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigate_to_goal_server = group2_gp2.scripts.main_navigate_to_goal_server:main',
        ],
    },
)