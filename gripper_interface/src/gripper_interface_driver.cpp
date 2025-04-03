#include "gripper_interface/gripper_interface_driver.hpp"
#include <cstdint>
#include <string>
#include "canfd.h"

GripperInterfaceDriver::GripperInterfaceDriver(std::string can_interface,
                                               int can_enabled,
                                               short i2c_bus,
                                               int i2c_address,
                                               int pwm_gain,
                                               int pwm_idle)
    : can_interface_(can_interface),
      can_enabled_(can_enabled),
      i2c_bus_(i2c_bus),
      i2c_address_(i2c_address),
      pwm_gain_(pwm_gain),
      pwm_idle_(pwm_idle) {
    if (can_enabled_) {
        if (canfd_init(can_interface_.c_str())) {
            throw std::runtime_error(
                std::format("ERROR: Failed to initialize CAN FD {} : {}",
                            can_interface_, strerror(errno)));
        }
    } else {
        std::string i2c_filename = std::format("/dev/i2c-{}", i2c_bus_);
        bus_fd_ =
            open(i2c_filename.c_str(),
                 O_RDWR);  // Open the I2C bus for reading and writing (O_RDWR)
        if (bus_fd_ < 0) {
            throw std::runtime_error(
                std::format("ERROR: Failed to open I2C bus {} : {}", i2c_bus_,
                            strerror(errno)));
        }
    }
}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    if (can_enabled_) {
        canfd_close();
    } else {
        if (bus_fd_ >= 0) {
            send_pwm(std::vector<std::uint16_t>(3, pwm_idle_));
            close(bus_fd_);
        }
    }
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle_ + pwm_gain_ * joy_value);
}

void GripperInterfaceDriver::send_pwm(
    const std::vector<std::uint16_t>& pwm_values) {
    try {
        constexpr std::size_t i2c_data_size =
            1 + 3 * 2;  // 3 thrusters * (1xMSB + 1xLSB)
        std::array<std::uint8_t, i2c_data_size> i2c_data_array;

        i2c_data_array.at(0) = 0x00;  // "Start" byte
        auto joined_bytes = pwm_values |
                            std::views::transform([](std::uint16_t pwm) {
                                return pwm_to_i2c_data(pwm);
                            }) |
                            std::views::join;

        std::ranges::copy(joined_bytes, i2c_data_array.begin() + 1); 

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error(std::format(
                "Failed to open I2C bus {} : {}", i2c_bus_, strerror(errno)));
            return;
        }

        if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            i2c_data_size) {
            throw std::runtime_error(std::format(
                "Error: Failed to write to I2C device : {}", strerror(errno)));
        }
    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send PWM values - {}", e.what());
    } catch (...) {
        spdlog::error("ERROR: Failed to send PWM values - unknown error");
    }
}

void GripperInterfaceDriver::send_pwm_can(
    const std::vector<std::uint16_t>& pwm_values) {
    try {
        CANFD_Message msg;
        msg.id = 0x46B;
        constexpr std::size_t can_len = 3 * 2;
        msg.length = static_cast<uint8_t>(can_len);
        msg.is_fd = true;
        msg.is_extended = false;

        auto joined_bytes = pwm_values |
                            std::views::transform([](std::uint16_t pwm) {
                                return pwm_to_i2c_data(pwm);
                            }) |
                            std::views::join;

        std::ranges::copy(joined_bytes, msg.data);
        if (canfd_send(&msg)) {
            throw std::runtime_error(std::format(
                "Error: Failed to send CAN message: {}", strerror(errno)));
        }

    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send PWM values - {}", e.what());
    } catch (...) {
        spdlog::error("ERROR: Failed to send PWM values - unknown error");
    }
}
void GripperInterfaceDriver::stop_gripper() {
    try {
        constexpr std::size_t i2c_data_size = 1;
        std::uint8_t i2c_message = 0x01;

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error(std::format(
                "Failed to open I2C bus {} : {}", i2c_bus_, strerror(errno)));
            return;
        }

        if (write(bus_fd_, &i2c_message, i2c_data_size) != i2c_data_size) {
            throw std::runtime_error(std::format(
                "Error: Failed to write to I2C device : {}", strerror(errno)));
        }
    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send stop gripper command - {}",
                      e.what());
    } catch (...) {
        spdlog::error(
            "ERROR: Failed to send stop gripper command - unknown error");
    }
}

void GripperInterfaceDriver::stop_gripper_can() {
    try {
        CANFD_Message msg;
        msg.id = 0x469;
        msg.is_extended = false;
        msg.is_fd = true;
        msg.data[0] = 0x00;
        msg.length = 1;

        if (canfd_send(&msg)) {
            throw std::runtime_error(std::format(
                "Error: Failed to send CAN message: {}", strerror(errno)));
        }

    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send stop gripper command - {}",
                      e.what());
    } catch (...) {
        spdlog::error(
            "ERROR: Failed to send stop gripper command - unknown error");
    }
}

void GripperInterfaceDriver::start_gripper() {
    try {
        constexpr std::size_t i2c_data_size = 1;
        std::uint8_t i2c_message = 0x02;

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error(std::format(
                "Failed to open I2C bus {} : {}", i2c_bus_, strerror(errno)));
            return;
        }

        if (write(bus_fd_, &i2c_message, i2c_data_size) != i2c_data_size) {
            throw std::runtime_error(std::format(
                "Error: Failed to write to I2C device : {}", strerror(errno)));
        }
    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send start gripper command - {}",
                      e.what());
    } catch (...) {
        spdlog::error(
            "ERROR: Failed to send start gripper command - unknown error");
    }
}

void GripperInterfaceDriver::start_gripper_can() {
    try {
        CANFD_Message msg;
        msg.id = 0x46A;
        msg.is_extended = false;
        msg.is_fd = true;
        msg.data[0] = 0x00;
        msg.length = 1;

        if (canfd_send(&msg)) {
            throw std::runtime_error(std::format(
                "Error: Failed to send CAN message: {}", strerror(errno)));
        }

    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send stop gripper command - {}",
                      e.what());
    } catch (...) {
        spdlog::error(
            "ERROR: Failed to send stop gripper command - unknown error");
    }
}

std::vector<double> GripperInterfaceDriver::encoder_read() {
    try {
        constexpr std::size_t i2c_data_size = 6;  // 6 bytes -> 3 angles.
        std::vector<std::uint8_t> i2c_data_array(i2c_data_size);
        std::vector<double> encoder_angles;
        encoder_angles.reserve(i2c_data_size / 2);

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error(std::format(
                "Failed to open I2C bus {} : {}", i2c_bus_, strerror(errno)));
        }

        if (read(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            static_cast<ssize_t>(i2c_data_size)) {
            throw std::runtime_error(std::format(
                "Error: Failed to read from I2C device : {}", strerror(errno)));
        }

        const std::size_t num_angles = i2c_data_size / 2;
        std::vector<std::size_t> indices(num_angles);
        std::iota(indices.begin(), indices.end(), 0);

        std::ranges::transform(
            indices, std::back_inserter(encoder_angles),
            [&](std::size_t idx) -> double {
                std::array<std::uint8_t, 2> pair = {
                    i2c_data_array[2 * idx], i2c_data_array[2 * idx + 1]};
                std::uint16_t raw_angle = i2c_to_encoder_angles(pair);
                return raw_angle_to_radians(raw_angle);
            });

        return encoder_angles;
    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to read encoder values - {}", e.what());
    } catch (...) {
        spdlog::error("ERROR: Failed to read encoder values - unknown error");
    }
    return {};
}
