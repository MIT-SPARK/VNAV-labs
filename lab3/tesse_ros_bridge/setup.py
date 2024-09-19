from setuptools import find_packages, setup
from glob import glob

package_name = 'tesse_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tesse_quadrotor_bridge.launch.yaml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm',
    maintainer_email='fishberg.dev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_teleop = key_teleop.key_teleop:main',
            'quadrotor_control_interface = tesse_ros_bridge.quadrotor_control_interface:main',
            'tesse_ros_node = tesse_ros_bridge.tesse_ros_node:main'
        ],
    },
)
