cmake_minimum_required(VERSION 3.8)
project(gui_app)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(backend REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(include)
set(HEADERS
  include/gui_app/mainwindow.hpp
)

set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
)

add_executable(gui_node ${SOURCES} ${HEADERS})


target_link_libraries(gui_node Qt5::Widgets)
ament_target_dependencies(gui_node rclcpp std_msgs std_srvs backend)

install(TARGETS gui_node DESTINATION lib/${PROJECT_NAME})


ament_package()
