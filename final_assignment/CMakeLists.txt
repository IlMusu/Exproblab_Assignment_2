cmake_minimum_required(VERSION 3.0.2)
project(final_assignment)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  aruco
  OpenCV
  cv_bridge
  armor_py_api
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
  actionlib_msgs
  robot_state_msgs
  actionlib
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    OntologyMap.msg
    RoomDoors.msg
    RoomConnection.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    RoomInformation.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    RobotInspectionRoutine.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    move_base_msgs
    actionlib_msgs
    robot_state_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES final_assignment
  CATKIN_DEPENDS 
    rospy
    roscpp
    aruco
    cv_bridge
    armor_py_api
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    move_base_msgs
    actionlib_msgs
    robot_state_msgs
    actionlib
    message_generation
  DEPENDS 
    OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(marker_server src/marker_server.cpp)
add_executable(marker_detector src/marker_detector.cpp)

## Add cmake target dependencies of the executable
add_dependencies(marker_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(marker_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(marker_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(marker_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

