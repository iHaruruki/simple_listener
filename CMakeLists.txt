cmake_minimum_required(VERSION 3.16)
project(simple_listener)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(YARP REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_listener src/main.cpp)
ament_target_dependencies(simple_listener rclcpp std_msgs geometry_msgs YARP)

#target_link_libraries(ros2_yarp_bridge YARP)

install(TARGETS
  simple_listener
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()