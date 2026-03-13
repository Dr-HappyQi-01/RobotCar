cmake_minimum_required(VERSION 3.0.2)
project(robot_monitor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

catkin_package()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(HEADERS
  include/robot_monitor/main_window.h
  include/robot_monitor/ros_interface.h
  include/robot_monitor/map_view_widget.h
)

set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/ros_interface.cpp
  src/map_view_widget.cpp
)

add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  uuid
)