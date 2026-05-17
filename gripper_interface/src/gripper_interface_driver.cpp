#include "gripper_interface/gripper_interface_driver.hpp"
#include "can_interface.hpp"

GripperInterfaceDriver::GripperInterfaceDriver(int pwm_gain, int pwm_idle)
    : pwm_gain_(pwm_gain), pwm_idle_(pwm_idle) {}

can_status GripperInterfaceDriver::init_can() {
    constexpr uint32_t ENCODER_ANGLES_CAN_ID = 0x46D;
    if (can_.init("can0") != can_status::OK) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    return can_.set_filter(ENCODER_ANGLES_CAN_ID);
}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    send_pwm(std::vector<std::uint16_t>(3, pwm_idle_));
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle_ + pwm_gain_ * joy_value);
}

can_status GripperInterfaceDriver::send_pwm(
    const std::vector<std::uint16_t>& pwm_values) {
    static constexpr uint32_t GRIPPER_PWM_CAN_ID = 0x46B;
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

void GripperInterfaceDriver::start_read_encoders(std::function<void(const struct canfd_frame&, can_status)> callback){
    can_.start_async_receive(callback);
}

std::vector<double> GripperInterfaceDriver::parse_encoders(const struct canfd_frame& frame) {
    constexpr std::size_t num_angles = 2;
    constexpr std::uint16_t invalid_reading = 0xFFFF;

    std::vector<double> encoder_angles;
    encoder_angles.reserve(num_angles);

    for (std::size_t i = 0; i < num_angles; ++i) {
        const std::uint8_t lsb = frame.data[2 * i];
        const std::uint8_t msb = frame.data[2 * i + 1];

        const std::uint16_t raw_angle =
            static_cast<std::uint16_t>(lsb) |
            (static_cast<std::uint16_t>(msb) << 8);

        if (raw_angle == invalid_reading) {
            encoder_angles.push_back(std::numeric_limits<double>::quiet_NaN());
            continue;
        }

        encoder_angles.push_back(raw_angle_to_radians(raw_angle));
    }

    return encoder_angles;
}
