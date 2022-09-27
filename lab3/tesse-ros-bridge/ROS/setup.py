#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     name='tesse_ros_bridge',
     version='0.0.1',
     description='TESSE/ROS bridge',
     packages=['tesse_ros_bridge', 'key_teleop'],
     package_dir={'': 'src', '': 'src'},
     install_requires={'tesse', 'empy >= 3.3.0', 'opencv-python >= 4.2', 'scipy >= 1.2.0',},
)

setup(**setup_args)
