cmake_minimum_required(VERSION 3.20)

project(gripper_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(gripper_controller
    src/main.cpp
    src/joystick.cpp
    src/gripper_interface_driver.cpp
)

target_include_directories(gripper_controller
    PRIVATE
        include
)

target_compile_definitions(gripper_controller
    PRIVATE
        BOOST_ERROR_CODE_HEADER_ONLY
)

target_link_libraries(gripper_controller
    PRIVATE
        pthread
)

target_compile_options(gripper_controller
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
