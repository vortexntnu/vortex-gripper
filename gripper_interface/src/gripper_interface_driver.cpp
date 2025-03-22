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

void GripperInterfaceDriver::encoder_read(
    std::vector<std::uint16_t>& encoder_raw_angle) {
    try {
        constexpr std::size_t i2c_data_size = 6;
        std::vector<std::uint8_t> i2c_data_array(i2c_data_size);

        // Set up I2C communication
        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error("Failed to open I2C bus " +
                                     std::to_string(i2c_bus_) + " : " +
                                     std::string(strerror(errno)));
        }

        // Read exactly i2c_data_size bytes into the vector
        if (read(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            i2c_data_size) {
            throw std::runtime_error(
                "Error: Failed to read from I2C device : " +
                std::string(strerror(errno)));
        }

        for (std::uint8_t i = 0; i < i2c_data_size; i += 2) {
            std::array<std::uint8_t, 2> pair = {i2c_data_array[i],
                                                i2c_data_array[i + 1]};
            std::uint16_t angle = i2c_to_encoder_angles(pair);
            encoder_raw_angle.push_back(angle);
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to read encoder angles - " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "ERROR: Failed to read encoder angles - unknown error"
                  << std::endl;
    }
}
