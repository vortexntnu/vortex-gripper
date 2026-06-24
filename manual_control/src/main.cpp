#include <vortex/gripper/gripper_driver.hpp>
#include <vortex/gripper/joystick.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> running{true};

void handle_signal(int) {
    running = false;
}

double apply_deadband(double value, double deadband) {
    if (std::abs(value) < deadband) {
        return 0.0;
    }

    const double sign = value < 0.0 ? -1.0 : 1.0;

    return sign * (std::abs(value) - deadband) / (1.0 - deadband);
}

std::uint16_t to_pwm(double joystick_value, int pwm_idle, int pwm_gain) {
    const int pwm = pwm_idle + static_cast<int>(pwm_gain * joystick_value);

    return static_cast<std::uint16_t>(std::clamp(pwm, 700, 2300));
}

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [joystick_device] [serial_port]\n\n"
              << "Defaults:\n"
              << "  joystick_device: /dev/input/js0\n"
              << "  serial_port:     /dev/ttyUSB0\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string_view(argv[1]) == "--help") {
        print_usage(argv[0]);
        return 0;
    }

    const std::string joystick_device = argc >= 2 ? argv[1] : "/dev/input/js0";

    const std::string serial_port = argc >= 3 ? argv[2] : "/dev/ttyUSB0";

    constexpr unsigned int serial_baudrate = 115200;

    constexpr int pwm_idle = 1500;
    constexpr int pwm_gain = 500;
    constexpr double joystick_deadband = 0.12;

    constexpr std::size_t shoulder_axis = 1;
    constexpr std::size_t wrist_axis = 4;

    constexpr std::size_t start_button = 0;
    constexpr std::size_t stop_button = 1;

    constexpr auto control_period = std::chrono::milliseconds(20);

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    vortex::gripper::Joystick joystick(joystick_device);

    if (!joystick.is_open()) {
        std::cerr << "Could not open joystick: " << joystick_device << '\n';
        return 1;
    }

    boost::asio::io_context io;

    vortex::gripper::GripperDriver driver(io, serial_port, serial_baudrate,
                                          pwm_gain, pwm_idle);

    const auto open_status = driver.init_serial();

    if (open_status != vortex::gripper::SerialStatus::OK) {
        std::cerr << "Could not open gripper serial port: " << serial_port
                  << '\n';
        return 1;
    }

    bool start_was_pressed = false;
    bool stop_was_pressed = false;

    std::cout << "Manual gripper control started\n";
    std::cout << "Joystick: " << joystick_device << '\n';
    std::cout << "Serial:   " << serial_port << '\n';

    auto next_tick = std::chrono::steady_clock::now();

    while (running) {
        joystick.poll();

        const double shoulder =
            apply_deadband(joystick.axis(shoulder_axis), joystick_deadband);

        const double wrist =
            apply_deadband(joystick.axis(wrist_axis), joystick_deadband);

        const std::uint16_t shoulder_pwm = to_pwm(shoulder, pwm_idle, pwm_gain);

        const std::uint16_t wrist_pwm = to_pwm(wrist, pwm_idle, pwm_gain);

        const std::uint16_t claw_pwm = static_cast<std::uint16_t>(pwm_idle);

        const bool start_pressed = joystick.button(start_button);
        const bool stop_pressed = joystick.button(stop_button);

        const bool start_rising_edge = start_pressed && !start_was_pressed;

        const bool stop_rising_edge = stop_pressed && !stop_was_pressed;

        start_was_pressed = start_pressed;
        stop_was_pressed = stop_pressed;

        if (start_rising_edge) {
            const auto status = driver.start_gripper();

            if (status != vortex::gripper::SerialStatus::OK) {
                std::cerr << "Failed to start gripper\n";
            } else {
                std::cout << "Gripper started\n";
            }
        }

        if (stop_rising_edge) {
            const auto status = driver.stop_gripper();

            if (status != vortex::gripper::SerialStatus::OK) {
                std::cerr << "Failed to stop gripper\n";
            } else {
                std::cout << "Gripper stopped\n";
            }
        }

        const auto pwm_status =
            driver.send_pwm(shoulder_pwm, wrist_pwm, claw_pwm);

        if (pwm_status != vortex::gripper::SerialStatus::OK) {
            std::cerr << "Failed to send PWM command\n";
        }

        next_tick += control_period;

        const auto now = std::chrono::steady_clock::now();

        if (next_tick < now) {
            next_tick = now + control_period;
        }

        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "\nStopping: commanding neutral PWM\n";

    driver.send_pwm(static_cast<std::uint16_t>(pwm_idle),
                    static_cast<std::uint16_t>(pwm_idle),
                    static_cast<std::uint16_t>(pwm_idle));

    driver.stop_gripper();

    return 0;
}
