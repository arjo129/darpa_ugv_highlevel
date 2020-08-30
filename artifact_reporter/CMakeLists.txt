cmake_minimum_required(VERSION 2.8.3)
project(artifact_reporter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2_msgs
  object_detection
  geometry_msgs
  noroute_mesh
)

find_package(Boost REQUIRED)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(artifact_report_spam src/report_artifact_spam.cpp)
add_dependencies(artifact_report_spam ${catkin_EXPORTED_TARGETS})

target_link_libraries(artifact_report_spam
  ${catkin_LIBRARIES}
)


