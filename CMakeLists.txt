cmake_minimum_required(VERSION 3.0.2)
project(ttc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ttc_calculator src/ttc_calculator.cpp)
target_link_libraries(ttc_calculator ${catkin_LIBRARIES})
add_dependencies(ttc_calculator ${catkin_EXPORTED_TARGETS})

## Build TTC Visualizer node
add_executable(ttc_visualizer src/ttc_visualizer.cpp)
target_link_libraries(ttc_visualizer ${catkin_LIBRARIES})
add_dependencies(ttc_visualizer ${catkin_EXPORTED_TARGETS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/ttc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)