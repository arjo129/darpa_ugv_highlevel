cmake_minimum_required(VERSION 2.8.3)
project(amapper)
set (CMAKE_CXX_STANDARD 11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  sensor_msgs
  tf2
  roscpp
  pcl_ros
  std_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
  rviz
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
set(CMAKE_AUTOMOC ON)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##NaiveRaytracer
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ElevationPoint.msg
  ElevationGridMsg.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  lightbuoydetector.srv
#  mapReset.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.actiorn
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs  # Or other packages containing msgs
 nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#  cfg/map.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amapper
#  CATKIN_DEPENDS asv_msgs sbg_driver
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${Qt5Core_INCLUDE_DIRS}
 ${Qt5Gui_INCLUDE_DIRS}
 ${Qt5Widgets_INCLUDE_DIRS}
 ${Qt5PrintSupport_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/grid.cpp
  src/${PROJECT_NAME}/elevation_grid.cpp
  src/${PROJECT_NAME}/elevation_iterator.cpp
  src/${PROJECT_NAME}/RayTracer.cpp
  src/${PROJECT_NAME}/cluster.cpp
  src/${PROJECT_NAME}/blur.cpp
  src/devices/laserscan.cpp
  src/raytracers/NaiveRayTracer.cpp
  src/raytracers/NoClearing.cpp
)

add_library(elevation_rviz
  src/rviz/ElevationDisplay.cpp
  src/rviz/ElevationVisual.cpp
  include/amapper/rviz/ElevationDisplay.h
  include/amapper/rviz/ElevationVisual.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(elevation_rviz ${PROJECT_NAME})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/main.cpp)
add_executable(laser_scan_example src/examples/local_laser_scan_based_map.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(laser_scan_example ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
)


target_link_libraries(laser_scan_example
   ${catkin_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
   ${PROJECT_NAME} 
)

target_link_libraries(elevation_rviz
   ${catkin_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
   ${PROJECT_NAME} 
   ${OGRE_LIBRARIES}
   Qt5::Core
   Qt5::Widgets
   Qt5::Gui
   Qt5::PrintSupport
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-grid-test test/test_grid.cpp)
if(TARGET ${PROJECT_NAME}-grid-test)
   target_link_libraries(${PROJECT_NAME}-grid-test ${PROJECT_NAME} ${catkin_LIBRARIES}
   ${PCL_COMMON_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
