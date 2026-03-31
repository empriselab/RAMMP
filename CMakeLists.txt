cmake_minimum_required(VERSION 3.0.2)
project(drink_actions_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  DIRECTORY action
  FILES
  DrinkAction.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    message_runtime
    rospy
    std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/rammp/integration/drink_action_server.py
  src/rammp/integration/drink_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()