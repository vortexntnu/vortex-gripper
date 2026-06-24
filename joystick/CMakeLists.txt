add_library(joystick STATIC
    src/joystick.cpp
)

target_include_directories(joystick
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(joystick
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
