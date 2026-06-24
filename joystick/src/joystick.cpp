#include "vortex/gripper/joystick.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

namespace vortex::gripper {

Joystick::Joystick(const std::string& device) {
    fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);

    if (fd_ < 0) {
        std::cerr << "Failed to open " << device
                  << ": " << std::strerror(errno) << '\n';
        return;
    }

    char controller_name[128] = "Unknown";
    ioctl(fd_, JSIOCGNAME(sizeof(controller_name)), controller_name);

    std::uint8_t axis_count = 0;
    std::uint8_t button_count = 0;

    ioctl(fd_, JSIOCGAXES, &axis_count);
    ioctl(fd_, JSIOCGBUTTONS, &button_count);

    std::cout << "Opened joystick: " << controller_name << '\n';
    std::cout << "Axes: " << static_cast<int>(axis_count)
              << ", buttons: " << static_cast<int>(button_count) << '\n';
}

Joystick::~Joystick() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool Joystick::is_open() const {
    return fd_ >= 0;
}

bool Joystick::poll() {
    if (fd_ < 0) {
        return false;
    }

    bool got_event = false;
    js_event event{};

    while (true) {
        const ssize_t bytes_read = read(fd_, &event, sizeof(event));

        if (bytes_read == static_cast<ssize_t>(sizeof(event))) {
            got_event = true;

            // Initial-state events have JS_EVENT_INIT set as well.
            const std::uint8_t type = event.type & ~JS_EVENT_INIT;

            if (type == JS_EVENT_AXIS && event.number < axes_.size()) {
                axes_[event.number] = event.value;
            } else if (type == JS_EVENT_BUTTON &&
                       event.number < buttons_.size()) {
                buttons_[event.number] = (event.value != 0);
            }

            continue;
        }

        if (bytes_read < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        }

        if (bytes_read == 0) {
            std::cerr << "Joystick disconnected\n";
            break;
        }

        if (bytes_read < 0) {
            std::cerr << "Joystick read error: "
                      << std::strerror(errno) << '\n';
            break;
        }

        break;
    }

    return got_event;
}

double Joystick::axis(std::size_t index) const {
    if (index >= axes_.size()) {
        return 0.0;
    }

    const auto raw = axes_[index];

    if (raw >= 0) {
        return static_cast<double>(raw) / 32767.0;
    }

    return static_cast<double>(raw) / 32768.0;
}

bool Joystick::button(std::size_t index) const {
    return index < buttons_.size() && buttons_[index];
}

} // namespace vortex::gripper
