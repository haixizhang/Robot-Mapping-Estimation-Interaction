cmake_minimum_required(VERSION 2.8.3)
project(perception)
# set include paths
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}")

# add /usr/share/cmake-2.8/Modules/ to CMAKE_MODULE_PATH
# CMAKE_MODULE_PATH define function, macros for CMakeList
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake-2.8/Modules/)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(/usr/lib/x86_64-linux-gnu/)
# APRILTAGS_LIBRARY: variable name for path
find_library(APRILTAGS_LIBRARY NAMES apriltags HINTS ${CMAKE_INSTALL_PREFIX}/
lib)
# message("CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}")
message("APRILTAGS_LIBRARY:${APRILTAGS_LIBRARY}")

add_message_files(
  FILES
  Observation.msg
  Observations.msg
  Landmarks.msg
  Landmark.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES perception
  # CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
  # DEPENDS system_lib
)
include_directories(include)
message("catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_library(perception
  src/${PROJECT_NAME}/perception.cc
)
add_dependencies(perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(perception_node src/perception_node.cc)
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(perception_node
  perception
  ${OpenCV_LIBS}
  ${APRILTAGS_LIBRARY}
  ${catkin_LIBRARIES}
)
# message("catkin_LIBRARIES ${catkin_LIBRARIES}")
