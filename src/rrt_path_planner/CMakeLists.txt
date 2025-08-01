# File: rrt_global_planner/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(rrt_global_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_core
  costmap_2d
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  pluginlib
  angles
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_global_planner
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    nav_core 
    costmap_2d 
    nav_msgs 
    geometry_msgs 
    visualization_msgs 
    tf2_ros 
    pluginlib
    angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rrt_global_planner
  src/rrt_global_planner.cpp
)

target_link_libraries(rrt_global_planner
  ${catkin_LIBRARIES}
)

install(TARGETS rrt_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES rrt_global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


