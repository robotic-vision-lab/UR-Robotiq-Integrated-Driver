cmake_minimum_required(VERSION 3.0.2)
project(rvl_robot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY xacro configs models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})