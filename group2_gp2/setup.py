from setuptools import find_packages, setup

package_name = 'group2_gp2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxykumd , namfacchetti',
    maintainer_email='maxyk@umd.edu, gfacchet@terpmail.umd.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'navigate_to_goal_client = group2_gp2.scripts.main_navigate_to_goal_client:main',
        ],
    },
)
