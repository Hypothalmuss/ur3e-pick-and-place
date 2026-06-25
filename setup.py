from setuptools import find_packages, setup

package_name = 'ur3e_perception'

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
    description='Merged perception node (ArUco + YOLOv8 + fusion)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = ur3e_perception.perception_node:main',
        ],
    },
)
