#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace vortex::gripper {

class Joystick {
public:
    static constexpr std::size_t MAX_AXES = 16;
    static constexpr std::size_t MAX_BUTTONS = 32;

    explicit Joystick(const std::string& device = "/dev/input/js0");
    ~Joystick();

    Joystick(const Joystick&) = delete;
    Joystick& operator=(const Joystick&) = delete;

    bool is_open() const;

    // Read every queued event and update stored axis/button state.
    bool poll();

    // Returns a value in [-1.0, 1.0].
    double axis(std::size_t index) const;

    bool button(std::size_t index) const;

private:
    int fd_{-1};

    std::array<std::int16_t, MAX_AXES> axes_{};
    std::array<bool, MAX_BUTTONS> buttons_{};
};

} // namespace vortex::gripper
