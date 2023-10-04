cmake_minimum_required(VERSION 2.8.3)
project(potential_gap)

## Compile as C++17, supported in ROS noetic and newer
add_compile_options(-std=c++17)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

# the entry in catkin_package could be optional (I am not fully sure about this)
#catkin_package(
#  DEPENDS
#    OpenMP
#)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  pluginlib
  dynamic_reconfigure
  egocircle
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  # benchmarking_tools
  trajectory_generator
  turtlebot_trajectory_generator
  pips_trajectory_msgs
  dynamic_reconfigure
  message_generation
  trajectory_based_local_planner
  pips
  pips_trajectory_testing
  turtlebot_trajectory_testing
  pips_egocylindrical
  pips_egocircle
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/pg.cfg
  cfg/CollisionChecker.cfg
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_message_files(
  FILES
  TrajPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR}
  LIBRARIES potential_gap_ros potential_gap_core potential_gap_planner
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  DEPENDS OpenMP yaml-cpp
  CATKIN_DEPENDS base_local_planner dynamic_reconfigure egocircle geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs pips_trajectory_msgs trajectory_generator turtlebot_trajectory_generator message_runtime trajectory_based_local_planner pips_egocircle
)


add_library(potential_gap_core
  src/robot_geo_parser.cpp
  src/planner.cpp
  src/trajectory_follower.cpp
  # src/potential_gap.cpp
  src/gap_utils.cpp
  
  src/potentialgap_config.cpp
  src/visualization.cpp
  src/goal_selector.cpp
  src/trajectory_scoring.cpp
  src/gap_trajectory_generator.cpp
  src/gap_manip.cpp
  src/trajectory_controller.cpp
  ) 
add_dependencies(potential_gap_core ${PROJECT_NAME}_gencfg)

target_compile_options(potential_gap_core PRIVATE ${OpenMP_FLAGS})
target_link_libraries(potential_gap_core
${catkin_LIBRARIES}
${OpenMP_LIBS}
${YAML_CPP_LIBRARIES}
)

add_library(potential_gap_planner
  src/potential_gap.cpp
  ) 
add_dependencies(potential_gap_planner ${PROJECT_NAME}_gencfg)

target_compile_options(potential_gap_planner PRIVATE ${OpenMP_FLAGS})
target_link_libraries(potential_gap_planner
potential_gap_core
${catkin_LIBRARIES}
${OpenMP_LIBS}
)

add_library(potential_gap_ros
  src/potential_gap_ros.cpp
  ) 
add_dependencies(potential_gap_ros ${PROJECT_NAME}_gencfg)

target_compile_options(potential_gap_ros PRIVATE ${OpenMP_FLAGS})
target_link_libraries(potential_gap_ros
potential_gap_core
${catkin_LIBRARIES}
${OpenMP_LIBS}
)
