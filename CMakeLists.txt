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
  image_transport
  cv_bridge   
)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(PkgConfig REQUIRED)
find_package(TIFF REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

catkin_package()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(HEADERS
  include/robot_monitor/main_window.h
  include/robot_monitor/ros_interface.h
  include/robot_monitor/map_view_widget.h
  include/robot_monitor/trajectory_types.h
  include/robot_monitor/experiment_config.h
  include/robot_monitor/trajectory_recorder.h
  include/robot_monitor/trajectory_storage.h
  include/robot_monitor/camera_view_widget.h
  include/robot_monitor/camera_worker.h
  include/robot_monitor/sparkline_widget.h
  include/robot_monitor/progress_pill_widget.h
  include/robot_monitor/metrics_panel_widget.h
  include/robot_monitor/pose_metric_card_widget.h
  include/robot_monitor/map_types.h
  include/robot_monitor/app_config.h
  include/robot_monitor/map_file_loader.h
  include/robot_monitor/reward_curve_widget.h
)

set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/ros_interface.cpp
  src/map_view_widget.cpp
  src/experiment_config.cpp
  src/trajectory_recorder.cpp
  src/trajectory_storage.cpp
  src/camera_view_widget.cpp
  src/camera_worker.cpp
  src/sparkline_widget.cpp
  src/progress_pill_widget.cpp
  src/metrics_panel_widget.cpp
  src/pose_metric_card_widget.cpp
  src/app_config.cpp
  src/map_file_loader.cpp
  src/reward_curve_widget.cpp
)

add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${SQLITE3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TIFF_LIBRARIES}
  yaml-cpp
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  uuid
)


add_executable(draw_circle src/TestDemo.cpp)

target_link_libraries(draw_circle ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})