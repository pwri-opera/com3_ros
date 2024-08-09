cmake_minimum_required(VERSION 3.5)
project(com3_msgs)

set(ROS1 FALSE)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "noetic")
    set(ROS1 TRUE)
  endif()
  if($ENV{ROS_DISTRO} STREQUAL "melodic")
    set(ROS1 TRUE)
  endif()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(ROS1)
  find_package(catkin REQUIRED)
  find_package(message_generation REQUIRED)
  add_message_files(
    FILES
    "ExcavatorCom3MachineSetting.msg"
    "JointCmd.msg"
    "FluidPressureArray.msg"
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
  )
  catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ExcavatorCom3MachineSetting.msg"
    "msg/JointCmd.msg"
    "msg/FluidPressureArray.msg"
    DEPENDENCIES std_msgs sensor_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
  )
  ament_package()
endif()