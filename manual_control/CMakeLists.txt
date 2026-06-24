add_executable(gripper_manual_control
    src/main.cpp
)

target_link_libraries(gripper_manual_control
    PRIVATE
        joystick
        vortex_gripper_driver
        pthread
)

target_compile_options(gripper_manual_control
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
