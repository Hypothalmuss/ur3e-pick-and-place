import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3e_sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
         (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'moveit'),
         glob('config/moveit/*.yaml') + glob('config/moveit/*.srdf') + glob('config/moveit/*.rviz')),
        (os.path.join('lib', package_name),
         ['scripts/scene_initializer']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Simulation bringup for UR3e',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scene_initializer = ur3e_sim_bringup.scene_initializer_node:main',
        ],
    },
)
