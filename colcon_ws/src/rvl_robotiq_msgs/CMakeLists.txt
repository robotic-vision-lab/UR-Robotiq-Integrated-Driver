cmake_minimum_required(VERSION 3.8)
project(rvl_robotiq_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# define message files
set(MSG_FILES
  "msg/Robotiq2FCommand.msg"
  "msg/Robotiq2FStatus.msg"
)

# define service files
set(SRV_FILES
  "srv/RobotiqRequestOpening.srv"
  "srv/RobotiqRequestPosition.srv"
  "srv/RobotiqRequestStatus.srv"
)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

# export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_generator_py)

ament_package()
