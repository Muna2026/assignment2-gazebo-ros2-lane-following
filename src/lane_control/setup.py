from setuptools import find_packages, setup

package_name = 'lane_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Assignment 2 Team',
    maintainer_email='student@example.com',
    description='Feedback controller for the Gazebo ROS 2 lane-following race.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_controller = lane_control.lane_controller:main',
        ],
    },
)
