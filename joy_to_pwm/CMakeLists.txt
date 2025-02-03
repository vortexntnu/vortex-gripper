cmake_minimum_required(VERSION 3.8)
project(joy_to_pwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_to_pwm
  src/gripper_interface.cpp
  src/gripper_interface_driver.cpp)

ament_target_dependencies(joy_to_pwm rclcpp sensor_msgs)

install(TARGETS
  joy_to_pwm
  DESTINATION lib/${PROJECT_NAME})

ament_package()
