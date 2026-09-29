# Called by catkin (catkin_python_setup in CMakeLists.txt); do not run directly.
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(
    packages=['unity_slam_example', 'unity_slam_example.perception'],
    package_dir={'': '.'},
))
