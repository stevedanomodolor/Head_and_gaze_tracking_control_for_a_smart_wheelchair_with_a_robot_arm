cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulation_alg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control_msgs
  visualization_msgs
  geometry_msgs
  # visp_ros
  message_generation
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  tf2_ros
  kdl_parser
  cv_bridge
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
  vision_command.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_manipulation_alg
# CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport kdl_parser message_generation roscpp rospy sensor_msgs std_msgs tf visp_bridge
#  DEPENDS system_lib
CATKIN_DEPENDS message_generation moveit_core moveit_visual_tools moveit_ros_planning_interface tf2_ros
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wheel_chair_head_control_manager_node src/wheel_chair_head_control_manager_node.cpp)
target_link_libraries(wheel_chair_head_control_manager_node ${catkin_LIBRARIES}   ${OpenCV_LIBS})





catkin_install_python(PROGRAMS scripts/tracker/eye_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
