cmake_minimum_required(VERSION 3.8)
project(gripper_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(spdlog REQUIRED)

# Global include — inherited by all targets in this directory AND add_subdirectory(test)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---------------------------------------------------------------------------
# Component library (composable node)
# ---------------------------------------------------------------------------
set(COMPONENT_LIB ${PROJECT_NAME}_component)

add_library(${COMPONENT_LIB} SHARED
  src/can_interface.cpp
  src/gripper_interface_ros.cpp
)

target_include_directories(${COMPONENT_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${COMPONENT_LIB} PUBLIC
  rclcpp
  rclcpp_components
  vortex_msgs
  vortex_utils_ros
  spdlog
)

rclcpp_components_register_node(
  ${COMPONENT_LIB}
  PLUGIN "vortex::interface::GripperInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ---------------------------------------------------------------------------
# Standalone executable
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_standalone
  src/gripper_interface_node.cpp
)

target_link_libraries(${PROJECT_NAME}_standalone PUBLIC
  ${COMPONENT_LIB}
  fmt::fmt
)
ament_target_dependencies(${PROJECT_NAME}_standalone PUBLIC rclcpp)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  ${COMPONENT_LIB}
  ${PROJECT_NAME}_standalone
  EXPORT export_${COMPONENT_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
