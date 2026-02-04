cmake_minimum_required(VERSION 3.0.2)
project(glass_detection_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/sonar_publisher.py
  scripts/speed_controller.py
  scripts/sensor_fusion.py
  scripts/glass_map_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)