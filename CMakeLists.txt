cmake_minimum_required(VERSION 3.0.2)
project(rosbrachiograph)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Declare ROS messages, services and actions #########################
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ServoPosition.msg
)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration #####################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Install ###########################################################
catkin_install_python(PROGRAMS
  src/brachiograph_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)