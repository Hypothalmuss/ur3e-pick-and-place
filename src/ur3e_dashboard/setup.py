from setuptools import find_packages, setup

package_name = 'ur3e_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', ['static/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Web dashboard (FastAPI + rosbridge)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard_server = ur3e_dashboard.dashboard_server:main',
        ],
    },
)
