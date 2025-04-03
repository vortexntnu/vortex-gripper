cmake_minimum_required(VERSION 3.8)
project(gripper_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/gripper_interface_node.cpp
  src/gripper_interface_driver.cpp
  src/canfd.c
)

target_link_libraries(${PROJECT_NAME}_node fmt::fmt)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  spdlog
  fmt
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
