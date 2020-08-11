cmake_minimum_required(VERSION 2.8.3)
project(exploration_goal)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  graph_msgs
  roscpp
)

find_package(Boost REQUIRED)
find_package (Eigen3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exploration_goal
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/exploration_goal.cpp
# )

#add_executable(exploration_goal
#                src/exploration_goal_node.cpp)

add_executable(global_graph_debug 
                src/global_graph_node.cpp)

#target_link_libraries(
#  ${catkin_LIBRARIES}
#  #${PROJECT_NAME}
#  Eigen3::Eigen
#)

target_link_libraries(global_graph_debug
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

catkin_add_gtest(${PROJECT_NAME}-graph_test test/test_graph_merging.cpp)
if(TARGET ${PROJECT_NAME}-graph_test)
  target_link_libraries(
    ${PROJECT_NAME}-graph_test 
    ${catkin_LIBRARIES}
    Eigen3::Eigen
  )
endif()

