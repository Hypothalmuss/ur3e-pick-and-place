from setuptools import setup

package_name = 'demo_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo_recorder.launch.py']),
        ('share/' + package_name + '/config', ['config/demo_recorder.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Records robot demonstration episodes during teleoperation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'demo_recorder_node = demo_recorder.demo_recorder_node:main',
        ],
    },
)
