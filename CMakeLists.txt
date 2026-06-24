cmake_minimum_required(VERSION 3.20)

project(gripper_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(gripper_driver)
add_subdirectory(joystick)
add_subdirectory(manual_control)
