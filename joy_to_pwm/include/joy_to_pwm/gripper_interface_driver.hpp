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

    GripperInterfaceDriver(short i2c_bus = 1, short i2c_address = 0x21);

    std::uint16_t joy_to_pwm(const double joy_value);
    int joy_gain_;

   private:
    int bus_fd_;       // File descriptor for I2C bus
    int i2c_bus_;      // I2C bus number
    int i2c_address_;  // I2C address of the microcontroller
    std::uint16_t idle_pwm_value_;

};  // class GripperInterfaceDriver

#endif  // GRIPPER_INTERFACE_DRIVER_HPP
