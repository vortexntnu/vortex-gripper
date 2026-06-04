#pragma once

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

enum class serial_status {
    OK,
    ERR_NOT_INITIALIZED,
    ERR_WRITE_FAILED,
    ERR_READ_FAILED,
    ERR_BAD_PACKET
};

class GripperInterfaceDriver {
public:
    GripperInterfaceDriver(
        boost::asio::io_context& io,
        std::string port,
        unsigned int baudrate,
        int pwm_gain,
        int pwm_idle
    );

    ~GripperInterfaceDriver();

    serial_status init_serial();

    std::uint16_t joy_to_pwm(double joy_value);

    serial_status send_pwm(const std::vector<std::uint16_t>& pwm_values);
    serial_status stop_gripper();
    serial_status start_gripper();

    void start_read_encoders(
        std::function<void(const std::vector<double>&, serial_status)> callback
    );

private:
    static constexpr std::uint8_t SOF = 0xAA;

    static constexpr std::uint16_t GRIPPER_STOP_ID   = 0x469;
    static constexpr std::uint16_t GRIPPER_START_ID  = 0x46A;
    static constexpr std::uint16_t GRIPPER_PWM_ID    = 0x46B;
    static constexpr std::uint16_t ENCODER_ANGLES_ID = 0x46D;

    int pwm_gain_;
    int pwm_idle_;

    boost::asio::io_context& io_;
    boost::asio::serial_port serial_;
    std::string port_;
    unsigned int baudrate_;

    std::array<std::uint8_t, 256> rx_buf_{};
    std::vector<std::uint8_t> rx_accumulator_;

    std::function<void(const std::vector<double>&, serial_status)> encoder_callback_;

    serial_status send_packet(std::uint16_t id, const std::uint8_t* data, std::size_t len);

    void start_async_read();
    void handle_received_bytes(std::size_t bytes_received);

    bool try_extract_packet(
        std::uint16_t& id,
        std::vector<std::uint8_t>& payload
    );

    static std::uint8_t checksum_xor(
        std::uint16_t id,
        const std::uint8_t* data,
        std::size_t len
    );

    static std::vector<double> parse_encoders_payload(
        const std::vector<std::uint8_t>& payload
    );

    static double raw_angle_to_radians(std::uint16_t raw_angle);
};
