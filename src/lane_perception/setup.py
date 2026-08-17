from setuptools import find_packages, setup

package_name = 'lane_perception'

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
    description='Camera-based lane-center estimation for the Gazebo ROS 2 race assignment.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_detector = lane_perception.lane_detector:main',
        ],
    },
)
