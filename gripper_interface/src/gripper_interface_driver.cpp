#include "gripper_interface/gripper_interface_driver.hpp"

GripperInterfaceDriver::GripperInterfaceDriver(short i2c_bus,
                                               int i2c_address,
                                               int pwm_gain,
                                               int pwm_idle)
    : i2c_bus_(i2c_bus),
      i2c_address_(i2c_address),
      pwm_gain_(pwm_gain),
      pwm_idle_(pwm_idle) {
    std::string i2c_filename = "/dev/i2c-" + std::to_string(i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd_ < 0) {
        std::runtime_error("ERROR: Failed to open I2C bus " +
                           std::to_string(i2c_bus_) + " : " +
                           std::string(strerror(errno)));
    }
    write(bus_fd_, &pwm_idle_, 2);
}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    if (bus_fd_ >= 0) {
        send_pwm(std::vector<std::uint16_t>(3, pwm_idle_));
        close(bus_fd_);
    }
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    // Convert the joystick value to a PWM value
    return static_cast<std::uint16_t>(pwm_idle_ + pwm_gain_ * joy_value);
}

void GripperInterfaceDriver::send_pwm(
    const std::vector<std::uint16_t>& pwm_values) {
    try {
        constexpr std::size_t i2c_data_size =
            1 + 3 * 2;  // 3 thrusters * (1xMSB + 1xLSB)
        std::vector<std::uint8_t> i2c_data_array;
        i2c_data_array.reserve(i2c_data_size);

        i2c_data_array.push_back(0x00);  // Start byte
        std::for_each(
            pwm_values.begin(), pwm_values.end(), [&](std::uint16_t pwm) {
                std::array<std::uint8_t, 2> bytes = pwm_to_i2c_data(pwm);
                std::copy(bytes.begin(), bytes.end(),
                          std::back_inserter(i2c_data_array));
            });

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error("Failed to open I2C bus " +
                                     std::to_string(i2c_bus_) + " : " +
                                     std::string(strerror(errno)));
            return;
        }

        if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            i2c_data_size) {
            throw std::runtime_error("Error: Failed to write to I2C device : " +
                                     std::string(strerror(errno)));
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send PWM values - " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "ERROR: Failed to send PWM values - unknown error"
                  << std::endl;
    }
}

void GripperInterfaceDriver::stop_gripper() {
    try {
        constexpr std::size_t i2c_data_size = 1;
        std::vector<std::uint8_t> i2c_data_array;
        i2c_data_array.reserve(i2c_data_size);

        i2c_data_array.push_back(0x01);  // Stop byte stop_gripper

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error("Failed to open I2C bus " +
                                     std::to_string(i2c_bus_) + " : " +
                                     std::string(strerror(errno)));
            return;
        }

        if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            i2c_data_size) {
            throw std::runtime_error("Error: Failed to write to I2C device : " +
                                     std::string(strerror(errno)));
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send stop gripper command - " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr
            << "ERROR: Failed to send stop gripper command - unknown error"
            << std::endl;
    }
}

void GripperInterfaceDriver::start_gripper() {
    try {
        constexpr std::size_t i2c_data_size = 1;
        std::vector<std::uint8_t> i2c_data_array;
        i2c_data_array.reserve(i2c_data_size);

        i2c_data_array.push_back(0x02);  // Start byte start_gripper

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error("Failed to open I2C bus " +
                                     std::to_string(i2c_bus_) + " : " +
                                     std::string(strerror(errno)));
            return;
        }

        if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            i2c_data_size) {
            throw std::runtime_error("Error: Failed to write to I2C device : " +
                                     std::string(strerror(errno)));
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send start gripper command - "
                  << e.what() << std::endl;
    } catch (...) {
        std::cerr
            << "ERROR: Failed to send start gripper command - unknown error"
            << std::endl;
    }
}

std::vector<double> GripperInterfaceDriver::encoder_read() {
    constexpr std::size_t i2c_data_size = 6;  // 6 bytes -> 3 angles.
    std::vector<std::uint8_t> i2c_data_array(i2c_data_size);
    std::vector<double> encoder_angles;
    encoder_angles.reserve(i2c_data_size / 2);

    // Set up I2C communication
    if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
        throw std::runtime_error("Failed to open I2C bus " +
                                 std::to_string(i2c_bus_) + " : " +
                                 std::string(strerror(errno)));
    }

    if (read(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
        static_cast<ssize_t>(i2c_data_size)) {
        throw std::runtime_error("Error: Failed to read from I2C device : " +
                                 std::string(strerror(errno)));
    }

    // Create an index vector for the number of angles (pairs)
    const std::size_t num_angles = i2c_data_size / 2;
    std::vector<std::size_t> indices(num_angles);
    std::iota(indices.begin(), indices.end(), 0);

    std::transform(
        indices.begin(), indices.end(), std::back_inserter(encoder_angles),
        [&](std::size_t idx) -> double {
            std::array<std::uint8_t, 2> pair = {i2c_data_array[2 * idx],
                                                i2c_data_array[2 * idx + 1]};
            std::uint16_t raw_angle = i2c_to_encoder_angles(pair);
            // Convert raw_angle (14-bit value) to radians.
            return raw_angle_to_radians(raw_angle);
        });

    return encoder_angles;
}
