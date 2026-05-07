cmake_minimum_required(VERSION 3.8)
project(lucia_vision)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

add_executable(zed_publisher src/zed_publisher.cpp)

target_include_directories(zed_publisher PUBLIC
  /usr/local/zed/include
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(zed_publisher
  /usr/local/zed/lib/libsl_zed.so
  /usr/local/zed/lib/libsl_ai.so
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(zed_publisher ${DEPENDENCIES})

install(TARGETS zed_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
