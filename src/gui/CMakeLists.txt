cmake_minimum_required(VERSION 3.0.2)
project(gui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_runtime
  nav_msgs
  sensor_msgs
  map_msgs
  roscpp
  rospy
  std_msgs
  perception
)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS program_options system filesystem REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
message("Qt5Widgets_INCLUDE_DIRS: ${Qt5Widgets_INCLUDE_DIRS}")

include_directories(
  /usr/include/x86_64-linux-gnu/qt5/
  ${GLUT_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${BOOST_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

qt_wrap_cpp(gui MOC_SRCS include/${PROJECT_NAME}/gui.h)
add_library(gui
  src/${PROJECT_NAME}/gui.cpp
  ${MOC_SRCS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   gormetry_msgs#   nav_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################




catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gui
#  CATKIN_DEPENDS perception
#  CATKIN_DEPENDS gormetry_msgs message_generation message_runtime nav_msgs roscpp rospy std_msgs perception
#  DEPENDS system_lib
)

include_directories(include)

add_dependencies(gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gui_node src/gui_node.cpp)

add_dependencies(gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gui_node
  gui
  ${perception_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  Qt5::Widgets
  Qt5::OpenGL
  # /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.12.8
  ${catkin_LIBRARIES}
)
get_target_property(QT_WIDGETS_LOCATION Qt5::Widgets LOCATION)
get_target_property(QT_OPENGL_LOCATION Qt5::OpenGL LOCATION)

# Print the location
message(STATUS "Qt5::Widgets location: ${QT_WIDGETS_LOCATION}")
message(STATUS "Qt5::OpenGL location: ${QT_OPENGL_LOCATION}")

