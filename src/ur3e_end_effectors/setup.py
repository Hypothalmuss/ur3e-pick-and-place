from setuptools import find_packages, setup

package_name = 'ur3e_end_effectors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='End effector controllers for the UR3e (gripper + vacuum)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_state_node = ur3e_end_effectors.nodes.gripper_state_node:main',
            'vacuum_controller_node = ur3e_end_effectors.nodes.vacuum_controller_node:main',
        ],
    },
)
