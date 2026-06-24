add_library(vortex_gripper_driver STATIC
    src/gripper_driver.cpp
)

add_library(vortex::gripper_driver ALIAS vortex_gripper_driver)

target_include_directories(vortex_gripper_driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(vortex_gripper_driver
    PUBLIC
        BOOST_ERROR_CODE_HEADER_ONLY
)

target_compile_options(vortex_gripper_driver
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
