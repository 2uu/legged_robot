cmake_minimum_required(VERSION 2.8.3)
project(legged_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  gazebo_ros_legged_robot
  gazebo_ros_wamot_drive
  
  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  camera_info_manager
  std_msgs
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_legged_robot_walking_plugin SHARED plugin/gazebo_legged_robot_walking_plugin.cc)
target_link_libraries(gazebo_legged_robot_walking_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_legged_robot plugin/gazebo_ros_legged_robot.cpp)
target_link_libraries(gazebo_ros_legged_robot gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#no links to gazebo_ros_utils cause this error
#: undefined symbol: _ZN6gazebo9GazeboRos19readCommonParameterEv

add_executable(trot_control_node src/trot_control_node.cpp)
target_link_libraries(trot_control_node
   ${catkin_LIBRARIES}
 )



