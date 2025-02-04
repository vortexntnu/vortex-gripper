#ifndef GRIPPER_INTERFACE_DRIVER_HPP
#define GRIPPER_INTERFACE_DRIVER_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

class GripperInterfaceDriver {
   public:
    ~GripperInterfaceDriver();

    GripperInterfaceDriver(short i2c_bus, int i2c_address, int pwm_gain, int pwm_idle);

    std::uint16_t joy_to_pwm(const double joy_value);

    void send_pwm(const std::vector<std::uint16_t> &pwm_values);

   private:
    int bus_fd_;       // File descriptor for I2C bus
    int i2c_bus_;      // I2C bus number
    int i2c_address_;  // I2C address of the microcontroller
    int pwm_gain_;
    int pwm_idle_;

    static constexpr std::array<std::uint8_t, 2> pwm_to_i2c_data(
        std::uint16_t pwm) {
            return {static_cast<std::uint8_t>((pwm >> 8) & 0xFF),
                    static_cast<std::uint8_t>(pwm & 0xFF)};
        }
};  // class GripperInterfaceDriver

#endif  // GRIPPER_INTERFACE_DRIVER_HPP
