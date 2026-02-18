cmake_minimum_required(VERSION 3.8)
project(backend)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-missing-field-initializers)
endif()


find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetInt16.srv"
  "srv/SetFloat32.srv"
  DEPENDENCIES std_msgs
)


include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541/build/src_generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541/plugins/include)

set(UA_ENABLE_DISCOVERY ON CACHE INTERNAL "" FORCE)
set(UA_ENABLE_HISTORIZING OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541 open62541)



add_executable(opc_bridge src/main.cpp
                    src/opcua_client.cpp
                    src/config.cpp
                    src/ros_bridge.cpp
              )


target_link_libraries(opc_bridge 
  yaml-cpp
  open62541
  open62541-plugins
  Threads::Threads
  m
)

ament_target_dependencies(opc_bridge 
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
)



rosidl_target_interfaces(opc_bridge
${PROJECT_NAME} "rosidl_typesupport_cpp")


#install(TARGETS test_server
    #DESTINATION lib/${PROJECT_NAME}
  #)

install(TARGETS opc_bridge
DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
