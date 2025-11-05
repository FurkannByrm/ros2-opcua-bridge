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


include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541/build/src_generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/yaml-cpp/include)


set(UA_ENABLE_HISTORIZING OFF CACHE BOOL "Disable historizing")
set(UA_ENABLE_DISCOVERY OFF CACHE BOOL "Disable discovery")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/open62541 open62541)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/yaml-cpp yaml-cpp)


add_executable(main src/main.cpp
                    src/config.cpp)


target_link_libraries(main 
  yaml-cpp
  open62541-object
  open62541-plugins
  Threads::Threads
  m
)


ament_target_dependencies(main 
  rclcpp
)


install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Package configuration
ament_package()