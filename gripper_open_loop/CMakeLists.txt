cmake_minimum_required(VERSION 3.8)
project(gripper_open_loop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

# ---------------------------------------------------------------------------
# 1. Component Library (Shared)
# ---------------------------------------------------------------------------
set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/gripper_open_loop_ros.cpp
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_action
  rclcpp_components
  vortex_msgs
  vortex_utils_ros
  spdlog
  fmt
)

# ---------------------------------------------------------------------------
# 2. Component Registration
# ---------------------------------------------------------------------------
rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "vortex::open_loop::GripperOpenLoopNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ---------------------------------------------------------------------------
# 3. Standalone Executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_standalone
  src/gripper_open_loop_node.cpp
)
target_link_libraries(${PROJECT_NAME}_standalone PUBLIC
  ${LIB_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_standalone PUBLIC
  rclcpp
)

ament_export_targets(export_${LIB_NAME})
install(TARGETS ${LIB_NAME} ${PROJECT_NAME}_standalone
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
