#include "gripper_interface/gripper_interface_driver.hpp"
#include <cstddef>

GripperInterfaceDriver::GripperInterfaceDriver(short i2c_bus,
                                               int i2c_address,
                                               int pwm_gain,
                                               int pwm_idle)
    : i2c_bus_(i2c_bus),
      i2c_address_(i2c_address),
      pwm_gain_(pwm_gain),
      pwm_idle_(pwm_idle) {}

int GripperInterfaceDriver::init_i2c() {
    std::string i2c_filename = std::format("/dev/i2c-{}", i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the I2C bus for reading and writing (O_RDWR)
    if (bus_fd_ < 0) {
        return bus_fd_;
    }

    if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
        return -1;
    }
    return 0;
}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    if (bus_fd_ >= 0) {
        send_pwm(std::vector<std::uint16_t>(3, pwm_idle_));
        close(bus_fd_);
    }
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle_ + pwm_gain_ * joy_value);
}

int GripperInterfaceDriver::send_pwm(
    const std::vector<std::uint16_t>& pwm_values) {
    constexpr std::size_t num_servos = 3;
    constexpr std::size_t i2c_data_size =
        1 + num_servos * 2;  // 3 thrusters * (1xMSB + 1xLSB)
    std::array<std::uint8_t, i2c_data_size> buf;

    buf[0] = 0x00;  // "Start" byte

    std::memcpy(buf.data() + 1, pwm_values.data(), i2c_data_size - 1);

    if (write(bus_fd_, buf.data(), i2c_data_size) != i2c_data_size) {
        return -1;
    }
    return 0;
}

int GripperInterfaceDriver::stop_gripper() {
    constexpr std::size_t i2c_data_size = 1;
    std::uint8_t i2c_message = 0x01;

    if (write(bus_fd_, &i2c_message, i2c_data_size) != i2c_data_size) {
        return -1;
    }
    return 0;
}

int GripperInterfaceDriver::start_gripper() {
    constexpr std::size_t i2c_data_size = 1;
    std::uint8_t i2c_message = 0x02;

    if (write(bus_fd_, &i2c_message, i2c_data_size) != i2c_data_size) {
        return -1;
    }
    return 0;
}

std::vector<double> GripperInterfaceDriver::read_encoders() {
    constexpr std::size_t i2c_data_size = 6;  // 6 bytes -> 3 angles.
    constexpr std::size_t num_angles = i2c_data_size / 2;
    std::array<std::uint8_t, i2c_data_size> i2c_data_array;
    std::vector<double> encoder_angles;
    encoder_angles.reserve(num_angles);

    if (read(bus_fd_, i2c_data_array.data(), i2c_data_size) != i2c_data_size) {
        return {};
    }

    for (std::size_t i = 0; i < num_angles; ++i) {
        std::array<std::uint8_t, 2> pair = {i2c_data_array[2 * i],
                                            i2c_data_array[2 * i + 1]};
        std::uint16_t raw_angle = i2c_to_encoder_angles(pair);
        encoder_angles.push_back(raw_angle_to_radians(raw_angle));
    }

    return encoder_angles;
}
