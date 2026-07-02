import os
from setuptools import find_packages, setup

package_name = 'ur3e_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'motion.launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Motion planning and behavior tree orchestration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_place_bt_node = ur3e_motion.pick_place_bt_node:main',
            'motion_executor = ur3e_motion.motion_executor:main',
            'pick_place_orchestrator = ur3e_motion.pick_place_orchestrator:main',
        ],
    },
)
