#ifndef GRIPPER_INTERFACE_DRIVER_HPP
#define GRIPPER_INTERFACE_DRIVER_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

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
     * @param u The joystick value.
     * @return The PWM value.
     */
    std::uint16_t u_to_pwm(const double u);

    /**
     * @brief Send PWM values to the gripper.
     * @param pwm_values The PWM values.
     */
    void send_pwm(const std::vector<std::uint16_t>& pwm_values);

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
};  // class GripperInterfaceDriver

#endif  // GRIPPER_INTERFACE_DRIVER_HPP
