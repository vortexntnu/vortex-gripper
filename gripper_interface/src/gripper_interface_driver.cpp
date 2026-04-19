#include "gripper_interface/gripper_interface_driver.hpp"
#include <cstddef>
#include "can_interface.hpp"

GripperInterfaceDriver::GripperInterfaceDriver(int pwm_gain, int pwm_idle)
    : pwm_gain_(pwm_gain), pwm_idle_(pwm_idle) {}

can_status GripperInterfaceDriver::init_can() {
    if (can_.init("can0") != can_status::OK) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    return can_.set_filter(0x46D);
}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    send_pwm(std::vector<std::uint16_t>(3, pwm_idle_));
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle_ + pwm_gain_ * joy_value);
}

can_status GripperInterfaceDriver::send_pwm(
    const std::vector<std::uint16_t>& pwm_values) {
    static constexpr uint32_t GRIPPER_PWM_CAN_ID = 0x46C;
    constexpr std::size_t num_servos = 3;
    constexpr std::size_t data_size =
        num_servos * 2;  // 3 thrusters * (1xMSB + 1xLSB)
    std::array<std::uint8_t, data_size> buf;

    std::memcpy(buf.data(), pwm_values.data(), data_size);

    return can_.send(GRIPPER_PWM_CAN_ID, buf.data(), data_size, true);
}

can_status GripperInterfaceDriver::stop_gripper() {
    static constexpr uint32_t GRIPPER_STOP_CAN_ID = 0x469;
    constexpr std::size_t data_size = 1;
    std::uint8_t data = 0x00;

    return can_.send(GRIPPER_STOP_CAN_ID, &data, data_size, true);
}

can_status GripperInterfaceDriver::start_gripper() {
    static constexpr uint32_t GRIPPER_START_CAN_ID = 0x46A;
    constexpr std::size_t data_size = 1;
    std::uint8_t data = 0x02;

    return can_.send(GRIPPER_START_CAN_ID, &data, data_size, true);
}

std::vector<double> GripperInterfaceDriver::read_encoders() {
    constexpr std::size_t num_angles = 2;
    std::vector<double> encoder_angles;
    encoder_angles.reserve(num_angles);

    canfd_frame encoder_data{};
    if (can_.receive(encoder_data, 1000) != can_status::OK) {
        return {};
    }

    // for (std::size_t i = 0; i < num_angles; ++i) {
        // std::uint16_t raw_angle =
        //     (encoder_data[2 * i] & 0xFF) | (encoder_data[2 * i + 1] << 8);

        // i2c_to_encoder_angles(pair);
        // encoder_angles.push_back(raw_angle_to_radians(raw_angle));
    // }

    return encoder_angles;
}
