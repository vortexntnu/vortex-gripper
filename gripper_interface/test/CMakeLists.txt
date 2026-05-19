cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
find_package(vortex_msgs REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)

add_executable(
    ${TEST_BINARY_NAME}
    test_gripper_interface_translator.cpp
)

# Inherit include dirs from the component lib (PUBLIC target_include_directories
# on COMPONENT_LIB propagates here) + the directory-level include_directories
# set in the parent CMakeLists.txt
target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${PROJECT_NAME}_component
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC vortex_msgs)

gtest_discover_tests(${TEST_BINARY_NAME})
