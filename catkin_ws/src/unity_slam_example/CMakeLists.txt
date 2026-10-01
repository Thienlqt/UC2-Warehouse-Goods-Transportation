cmake_minimum_required(VERSION 3.0.2)
project(unity_slam_example)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  nodes/tf_odometry
  nodes/lidar_obstacles
  nodes/camera_detector
  nodes/obstacle_fusion
  nodes/cmd_vel_guard
  nodes/predicted_obstacles
  nodes/teleop_assist
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch models rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
