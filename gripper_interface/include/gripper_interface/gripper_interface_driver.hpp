#ifndef GRIPPER_INTERFACE_DRIVER_HPP
#define GRIPPER_INTERFACE_DRIVER_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <spdlog/spdlog.h>
#include <ranges>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <algorithm>  // for std::transform
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <numeric>  // for std::iota
#include <string>
#include <vector>
#include <format>

/**
 * @brief Class for interfacing with the gripper.
 */
class GripperInterfaceDriver {
   public:
    ~GripperInterfaceDriver();

    /**
     * @brief Constructor for the GripperInterfaceDriver class.
     * @param i2c_bus The I2C bus number.
     * @param i2c_address The I2C address of the microcontroller.
     * @param pwm_gain The gain for converting joystick values to PWM values.
     * @param pwm_idle The idle PWM value.
     */
    GripperInterfaceDriver(short i2c_bus,
                           int i2c_address,
                           int pwm_gain,
                           int pwm_idle);

    /**
     * @brief Convert joystick value to PWM value.
     * @param joy_value The joystick value.
     * @return The PWM value.
     */
    std::uint16_t joy_to_pwm(const double joy_value);

    /**
     * @brief Send PWM values to the gripper.
     * @param pwm_values The PWM values.
     */
    void send_pwm(const std::vector<std::uint16_t>& pwm_values);

    /**
     * @brief Start gripper by sending 0x02 start byte
     * @param None
     */
    void start_gripper();

    /**
     * @brief Stop gripper by sending 0x01 start byte
     * @param None
     */
    void stop_gripper();

    /**
     * @brief Stop gripper by sending 0x01 start byte
     * @param None
     */

    std::vector<double> encoder_read();

   private:
    int bus_fd_;       // File descriptor for I2C bus
    int i2c_bus_;      // I2C bus number
    int i2c_address_;  // I2C address of the microcontroller
    int pwm_gain_;
    int pwm_idle_;

    /**
     * @brief Convert PWM value to I2C data.
     * @param pwm The PWM value.
     * @return The I2C data.
     */
    static constexpr std::array<std::uint8_t, 2> pwm_to_i2c_data(
        std::uint16_t pwm) {
        return {static_cast<std::uint8_t>((pwm >> 8) & 0xFF),
                static_cast<std::uint8_t>(pwm & 0xFF)};
    }

    static constexpr std::uint16_t i2c_to_encoder_angles(
        std::array<std::uint8_t, 2> data) {
        return (static_cast<std::uint16_t>(data[0]) << 8) | data[1];
    }
    static constexpr double raw_angle_to_radians(std::uint16_t raw_angle) {
        return (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * M_PI);
    }
};  // class GripperInterfaceDriver

#endif  // GRIPPER_INTERFACE_DRIVER_HPP
