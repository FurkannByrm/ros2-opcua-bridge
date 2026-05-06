cmake_minimum_required(VERSION 3.8)
project(demonstrator_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)
add_executable(demo src/main.cpp 
                    src/demonstrator_tree/behavior_node.cpp
                    src/demonstrator_tree/parameters_parser.cpp
              )


target_link_libraries(demo 
  yaml-cpp
)

set(TARGET_DEPENDS
  rclcpp
  sensor_msgs
  std_srvs
  xbot_msgs
  behaviortree_cpp
  ament_index_cpp
)


ament_target_dependencies(demo 
${TARGET_DEPENDS}
)

install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config
)
ament_package()
