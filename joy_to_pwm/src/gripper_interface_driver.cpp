#include "joy_to_pwm/gripper_interface_driver.hpp"

GripperInterfaceDriver::GripperInterfaceDriver(
    short i2c_bus,
    short i2c_address
) : i2c_bus_(i2c_bus), i2c_address_(i2c_address)
{
    std::string i2c_filename = "/dev/i2c-" + std::to_string(i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd_ < 0) {
        std::runtime_error("ERROR: Failed to open I2C bus " +
                           std::to_string(i2c_bus_) + " : " +
                           std::string(strerror(errno)));
    }
    // Set the idle PWM value to 1500
    idle_pwm_value_ = 1500;
    write(bus_fd_, &idle_pwm_value_, 2);
}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    close(bus_fd_);
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    // Convert the joystick value to a PWM value
    return static_cast<std::uint16_t>(1500 + joy_gain_ * joy_value);
}