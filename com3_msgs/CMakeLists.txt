cmake_minimum_required(VERSION 3.0.2)
project(com3_msgs)

find_package(catkin REQUIRED std_msgs message_generation)

add_message_files(
  FILES
  excavator_com3_lever_1.msg
  excavator_com3_lever_2.msg
  excavator_com3_machine_setting.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES com3_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
