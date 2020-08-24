cmake_minimum_required(VERSION 2.8.3)
project(artifact_reporter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2_msgs
  object_detection
  subt_msgs
  geometry_msgs
)

# add_dependencies(subt_msgs ${CATKIN_EXPORTED_TARGETS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES artifact_reporter
#  CATKIN_DEPENDS roscpp rospy tf2 tf2_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/report_artifact_spam.cpp)


