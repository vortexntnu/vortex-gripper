#include <memory>
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
    const double magnitude = (std::abs(value) - deadband) / (1.0 - deadband);

    return sign * magnitude;
}

std::uint16_t clamp_pwm(int value) {
    constexpr int min_pwm = 700;
    constexpr int max_pwm = 2300;

    return static_cast<std::uint16_t>(std::clamp(value, min_pwm, max_pwm));
}

}  // namespace

struct ControlState {
    double shoulder{0.0};
    double wrist{0.0};

    bool start{false};
    bool stop{false};
    bool y{false};
};

ControlState simulated_control_state(
    std::chrono::steady_clock::duration elapsed) {
    using namespace std::chrono;

    const double t = duration<double>(elapsed).count();

    ControlState state{};

    /*
     * 0-2 s: neutral
     * 2-5 s: shoulder forward
     * 5-8 s: wrist backward
     * 8 s: one Start press
     * 10 s: one Y press, initiating timed rotation
     * 13 s: one Stop press
     */
    if (t >= 2.0 && t < 5.0) {
        state.shoulder = 0.60;
    }

    if (t >= 5.0 && t < 8.0) {
        state.wrist = -0.50;
    }

    if (t >= 8.0 && t < 8.1) {
        state.start = true;
    }

    if (t >= 10.0 && t < 10.1) {
        state.y = true;
    }

    if (t >= 13.0 && t < 13.1) {
        state.stop = true;
    }

    return state;
}

int main(int argc, char** argv) {
    const bool simulate = argc > 1 && std::string(argv[1]) == "--simulate";

    const std::string serial_port =
        simulate ? (argc > 2 ? argv[2] : "/tmp/gripper_serial")
                 : (argc > 2 ? argv[2] : "/dev/ttyUSB0");

    const std::string joystick_device =
        simulate ? "" : (argc > 1 ? argv[1] : "/dev/input/js0");

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

    std::unique_ptr<Joystick> joystick;

    if (!simulate) {
        joystick = std::make_unique<Joystick>(joystick_device);

        if (!joystick->is_open()) {
            return 1;
        }
    } else {
        std::cout << "Running with simulated joystick input\n";
    }

    boost::asio::io_context asio_io;

    GripperInterfaceDriver driver(asio_io, serial_port, serial_baudrate,
                                  pwm_gain, pwm_idle);

    const auto init_status = driver.init_serial();

    if (init_status != serial_status::OK) {
        std::cerr << "Failed to initialize serial on " << serial_port << ": "
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
                          << " rad, grip=" << angles[1] << " rad\n";
            }
        });

    std::thread asio_thread([&asio_io]() { asio_io.run(); });

    bool start_button_was_pressed = false;
    bool stop_button_was_pressed = false;
    bool y_button_was_pressed = false;

    bool rotate_90_active = false;
    auto rotate_90_end_time = std::chrono::steady_clock::time_point{};

    std::cout << "Controller started.\n";
    std::cout << "Joystick: " << joystick_device << '\n';
    std::cout << "Serial: " << serial_port << '\n';

    auto next_tick = std::chrono::steady_clock::now();
    const auto program_start = std::chrono::steady_clock::now();

    while (running) {
        const auto now = std::chrono::steady_clock::now();

        double shoulder_value = 0.0;
        double wrist_value = 0.0;

        bool y_pressed = false;
        bool start_pressed = false;
        bool stop_pressed = false;

        if (simulate) {
            const auto elapsed = now - program_start;
            const ControlState simulated = simulated_control_state(elapsed);

            // The simulator returns normalized values already.
            shoulder_value = simulated.shoulder;
            wrist_value = simulated.wrist;

            y_pressed = simulated.y;
            start_pressed = simulated.start;
            stop_pressed = simulated.stop;
        } else {
            joystick->poll();

            shoulder_value = apply_deadband(joystick->axis(shoulder_axis),
                                            joystick_deadband);

            wrist_value =
                apply_deadband(joystick->axis(wrist_axis), joystick_deadband);

            y_pressed = joystick->button(y_button);
            start_pressed = joystick->button(start_button);
            stop_pressed = joystick->button(stop_button);
        }

        std::vector<std::uint16_t> pwm_values = {
            clamp_pwm(pwm_idle + static_cast<int>(pwm_gain * shoulder_value)),

            clamp_pwm(pwm_idle + static_cast<int>(pwm_gain * wrist_value)),

            neutral_pwm,
        };

        // Y button starts a one-second timed rotation.
        const bool y_rising_edge = y_pressed && !y_button_was_pressed;
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

        // Start / stop are edge-triggered: one serial packet per press.
        const bool start_rising_edge =
            start_pressed && !start_button_was_pressed;

        const bool stop_rising_edge = stop_pressed && !stop_button_was_pressed;

        start_button_was_pressed = start_pressed;
        stop_button_was_pressed = stop_pressed;

        if (start_rising_edge) {
            const auto status = driver.start_gripper();

            std::cout << "Start gripper: " << serial_status_to_string(status)
                      << '\n';
        }

        if (stop_rising_edge) {
            const auto status = driver.stop_gripper();

            std::cout << "Stop gripper: " << serial_status_to_string(status)
                      << '\n';
        }

        // Always transmit at the control rate, even when joystick state is
        // unchanged.
        const auto pwm_status = driver.send_pwm(pwm_values);

        if (pwm_status != serial_status::OK) {
            std::cerr << "send_pwm failed: "
                      << serial_status_to_string(pwm_status) << '\n';
        }

        // Optional readable output while testing.
        static auto last_log = now;

        if (now - last_log >= std::chrono::milliseconds(500)) {
            std::cout << "PWM: " << pwm_values[0] << " " << pwm_values[1] << " "
                      << pwm_values[2] << " | rotate=" << rotate_90_active
                      << '\n';

            last_log = now;
        }

        next_tick += control_period;

        // Avoid a runaway loop if debugging or serial writes take too long.
        if (next_tick < now) {
            next_tick = now + control_period;
        }

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
