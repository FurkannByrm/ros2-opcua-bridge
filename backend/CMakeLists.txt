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

# ── open62541 (OPC UA library, must be installed system-wide) ──
find_package(open62541 REQUIRED)


add_executable(opc_bridge src/main.cpp
                    src/opcua_client.cpp
                    src/config.cpp
                    src/ros_bridge.cpp
              )


target_link_libraries(opc_bridge 
  yaml-cpp
  open62541::open62541
  Threads::Threads
)

ament_target_dependencies(opc_bridge 
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
)



rosidl_target_interfaces(opc_bridge
${PROJECT_NAME} "rosidl_typesupport_cpp")


# ── Test OPC UA Server (simulates PLC for development) ──
add_executable(test_server test/test_server.cpp)
target_link_libraries(test_server
  open62541::open62541
  Threads::Threads
)

install(TARGETS test_server
    DESTINATION lib/${PROJECT_NAME}
  )

install(TARGETS opc_bridge
DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
