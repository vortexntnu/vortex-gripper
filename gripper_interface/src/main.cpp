#include "gripper_interface/gripper_interface_driver.hpp"
#include "joystick.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> running{true};

void signal_handler(int) {
    running = false;
}

const char* serial_status_to_string(serial_status status) {
    switch (status) {
        case serial_status::OK:
            return "OK";
        case serial_status::ERR_NOT_INITIALIZED:
            return "ERR_NOT_INITIALIZED";
        case serial_status::ERR_WRITE_FAILED:
            return "ERR_WRITE_FAILED";
        case serial_status::ERR_READ_FAILED:
            return "ERR_READ_FAILED";
        case serial_status::ERR_BAD_PACKET:
            return "ERR_BAD_PACKET";
        default:
            return "UNKNOWN";
    }
}

double apply_deadband(double value, double deadband) {
    if (std::abs(value) < deadband) {
        return 0.0;
    }

    const double sign = value > 0.0 ? 1.0 : -1.0;
    const double magnitude =
        (std::abs(value) - deadband) / (1.0 - deadband);

    return sign * magnitude;
}

std::uint16_t clamp_pwm(int value) {
    constexpr int min_pwm = 700;
    constexpr int max_pwm = 2300;

    return static_cast<std::uint16_t>(
        std::clamp(value, min_pwm, max_pwm));
}

}  // namespace

int main(int argc, char** argv) {
    const std::string joystick_device =
        argc > 1 ? argv[1] : "/dev/input/js0";

    const std::string serial_port =
        argc > 2 ? argv[2] : "/dev/ttyUSB0";

    constexpr unsigned int serial_baudrate = 115200;

    constexpr int pwm_gain = 500;
    constexpr int pwm_idle = 1500;

    constexpr std::size_t shoulder_axis = 1;
    constexpr std::size_t wrist_axis = 4;

    constexpr std::size_t start_button = 0;
    constexpr std::size_t stop_button = 1;
    constexpr std::size_t y_button = 3;

    constexpr std::size_t rotate_pwm_index = 1;

    constexpr std::uint16_t neutral_pwm = 1500;
    constexpr std::uint16_t rotate_pwm = 1765;

    constexpr double joystick_deadband = 0.12;
    constexpr auto rotate_duration = std::chrono::seconds(1);

    // 50 Hz matches a typical servo control/command heartbeat rate.
    constexpr auto control_period = std::chrono::milliseconds(20);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    Joystick joystick(joystick_device);

    if (!joystick.is_open()) {
        return 1;
    }

    boost::asio::io_context asio_io;

    GripperInterfaceDriver driver(
        asio_io,
        serial_port,
        serial_baudrate,
        pwm_gain,
        pwm_idle);

    const auto init_status = driver.init_serial();

    if (init_status != serial_status::OK) {
        std::cerr << "Failed to initialize serial on "
                  << serial_port << ": "
                  << serial_status_to_string(init_status) << '\n';
        return 1;
    }

    // Start encoder reception before running io_context.
    driver.start_read_encoders(
        [](const std::vector<double>& angles, serial_status status) {
            if (status != serial_status::OK) {
                std::cerr << "Encoder read failed: "
                          << serial_status_to_string(status) << '\n';
                return;
            }

            if (angles.size() >= 2) {
                std::cout << "Encoders: wrist=" << angles[0]
                          << " rad, grip=" << angles[1]
                          << " rad\n";
            }
        });

    std::thread asio_thread([&asio_io]() {
        asio_io.run();
    });

    bool start_button_was_pressed = false;
    bool stop_button_was_pressed = false;
    bool y_button_was_pressed = false;

    bool rotate_90_active = false;
    auto rotate_90_end_time = std::chrono::steady_clock::time_point{};

    std::cout << "Controller started.\n";
    std::cout << "Joystick: " << joystick_device << '\n';
    std::cout << "Serial: " << serial_port << '\n';

    auto next_tick = std::chrono::steady_clock::now();

    while (running) {
        joystick.poll();

        const auto now = std::chrono::steady_clock::now();

        const double shoulder_value = apply_deadband(
            joystick.axis(shoulder_axis),
            joystick_deadband);

        const double wrist_value = apply_deadband(
            joystick.axis(wrist_axis),
            joystick_deadband);

        std::vector<std::uint16_t> pwm_values = {
            clamp_pwm(pwm_idle +
                      static_cast<int>(pwm_gain * shoulder_value)),

            clamp_pwm(pwm_idle +
                      static_cast<int>(pwm_gain * wrist_value)),

            neutral_pwm,
        };

        // Y button starts a one-second timed rotation.
        const bool y_pressed = joystick.button(y_button);
        const bool y_rising_edge =
            y_pressed && !y_button_was_pressed;

        y_button_was_pressed = y_pressed;

        if (y_rising_edge && !rotate_90_active) {
            rotate_90_active = true;
            rotate_90_end_time = now + rotate_duration;

            std::cout << "Y pressed: rotating 90 degrees\n";
        }

        if (rotate_90_active) {
            if (now < rotate_90_end_time) {
                pwm_values[rotate_pwm_index] = rotate_pwm;
            } else {
                rotate_90_active = false;
                pwm_values[rotate_pwm_index] = neutral_pwm;

                std::cout << "Timed rotation complete\n";
            }
        }

        // Start / stop rising edges.
        const bool start_pressed = joystick.button(start_button);
        const bool stop_pressed = joystick.button(stop_button);

        const bool start_rising_edge =
            start_pressed && !start_button_was_pressed;

        const bool stop_rising_edge =
            stop_pressed && !stop_button_was_pressed;

        start_button_was_pressed = start_pressed;
        stop_button_was_pressed = stop_pressed;

        if (start_rising_edge) {
            const auto status = driver.start_gripper();

            std::cout << "Start gripper: "
                      << serial_status_to_string(status) << '\n';
        }

        if (stop_rising_edge) {
            const auto status = driver.stop_gripper();

            std::cout << "Stop gripper: "
                      << serial_status_to_string(status) << '\n';
        }

        // Send regardless of whether a new event appeared.
        // This is the direct replacement for the ROS joy stream and
        // prevents a command watchdog from timing out while sticks are held.
        const auto pwm_status = driver.send_pwm(pwm_values);

        if (pwm_status != serial_status::OK) {
            std::cerr << "send_pwm failed: "
                      << serial_status_to_string(pwm_status) << '\n';
        }

        next_tick += control_period;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "\nShutting down: sending neutral PWM\n";

    driver.send_pwm({
        neutral_pwm,
        neutral_pwm,
        neutral_pwm,
    });

    driver.stop_gripper();

    asio_io.stop();

    if (asio_thread.joinable()) {
        asio_thread.join();
    }

    return 0;
}
