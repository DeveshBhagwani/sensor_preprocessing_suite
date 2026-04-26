cmake_minimum_required(VERSION 3.8)
project(sensor_preprocessing_suite)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(include)

add_library(sensor_preprocessing_core SHARED
  src/sensor_preprocessing_core.cpp
)

ament_target_dependencies(sensor_preprocessing_core
  sensor_msgs
)

add_library(sensor_preprocessing_component SHARED
  src/sensor_preprocessing_component.cpp
)

target_link_libraries(sensor_preprocessing_component
  sensor_preprocessing_core
)

ament_target_dependencies(sensor_preprocessing_component
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
)

rclcpp_components_register_nodes(sensor_preprocessing_component
  "sensor_preprocessing_suite::SensorPreprocessingComponent"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    sensor_preprocessing_core
    sensor_preprocessing_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
