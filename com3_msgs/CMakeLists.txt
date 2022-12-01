# find_package(catkin REQUIRED std_msgs message_generation)

# add_message_files(
#   FILES
#   excavator_com3_lever_1.msg
#   excavator_com3_lever_2.msg
#   excavator_com3_machine_setting.msg
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES com3_msgs
#   CATKIN_DEPENDS message_runtime
# #  DEPENDS system_lib
# )

# include_directories(
# # include
# # ${catkin_INCLUDE_DIRS}
# )

cmake_minimum_required(VERSION 3.5)
project(com3_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExcavatorCom3Lever1.msg"
  "msg/ExcavatorCom3Lever2.msg"
  "msg/ExcavatorCom3MachineSetting.msg"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()