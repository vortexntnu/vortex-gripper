#include "gripper_interface/gripper_interface_driver.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

GripperInterfaceDriver::GripperInterfaceDriver(boost::asio::io_context& io,
                                               std::string port,
                                               unsigned int baudrate,
                                               int pwm_gain,
                                               int pwm_idle)
    : pwm_gain_(pwm_gain),
      pwm_idle_(pwm_idle),
      io_(io),
      serial_(io),
      port_(std::move(port)),
      baudrate_(baudrate) {}

GripperInterfaceDriver::~GripperInterfaceDriver() {
    send_pwm(std::vector<std::uint16_t>(3, pwm_idle_));

    if (serial_.is_open()) {
        boost::system::error_code ec;
        serial_.cancel(ec);
        serial_.close(ec);
    }
}

serial_status GripperInterfaceDriver::init_serial() {
    boost::system::error_code ec;

    serial_.open(port_, ec);
    if (ec) {
        return serial_status::ERR_NOT_INITIALIZED;
    }

    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_), ec);
    if (ec) {
        return serial_status::ERR_NOT_INITIALIZED;
    }

    serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
    serial_.set_option(
        boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
        ec);
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
                           boost::asio::serial_port_base::stop_bits::one),
                       ec);
    serial_.set_option(boost::asio::serial_port_base::flow_control(
                           boost::asio::serial_port_base::flow_control::none),
                       ec);

    if (ec) {
        return serial_status::ERR_NOT_INITIALIZED;
    }

    return serial_status::OK;
}

std::uint16_t GripperInterfaceDriver::joy_to_pwm(const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle_ + pwm_gain_ * joy_value);
}

serial_status GripperInterfaceDriver::send_packet(std::uint16_t id,
                                                  const std::uint8_t* data,
                                                  std::size_t len) {
    if (!serial_.is_open()) {
        return serial_status::ERR_NOT_INITIALIZED;
    }

    if (len > 255) {
        return serial_status::ERR_BAD_PACKET;
    }

    std::vector<std::uint8_t> packet;
    packet.reserve(1 + 2 + 1 + len + 1);

    packet.push_back(SOF);

    packet.push_back(static_cast<std::uint8_t>(id & 0xFF));
    packet.push_back(static_cast<std::uint8_t>((id >> 8) & 0xFF));

    packet.push_back(static_cast<std::uint8_t>(len));

    packet.insert(packet.end(), data, data + len);

    packet.push_back(checksum_xor(id, data, len));

    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(packet), ec);

    if (ec) {
        return serial_status::ERR_WRITE_FAILED;
    }

    return serial_status::OK;
}

std::uint8_t GripperInterfaceDriver::checksum_xor(std::uint16_t id,
                                                  const std::uint8_t* data,
                                                  std::size_t len) {
    std::uint8_t cs = 0;

    cs ^= static_cast<std::uint8_t>(id & 0xFF);
    cs ^= static_cast<std::uint8_t>((id >> 8) & 0xFF);
    cs ^= static_cast<std::uint8_t>(len);

    for (std::size_t i = 0; i < len; ++i) {
        cs ^= data[i];
    }

    return cs;
}

void GripperInterfaceDriver::start_read_encoders(
    std::function<void(const std::vector<double>&, serial_status)> callback) {
    encoder_callback_ = std::move(callback);
    start_async_read();
}

void GripperInterfaceDriver::start_async_read() {
    serial_.async_read_some(
        boost::asio::buffer(rx_buf_),
        [this](const boost::system::error_code& ec, std::size_t bytes_received) {
            if (ec) {
                if (encoder_callback_) {
                    encoder_callback_({}, serial_status::ERR_READ_FAILED);
                }
                return;
            }

            handle_received_bytes(bytes_received);
            start_async_read();
        });
}

void GripperInterfaceDriver::handle_received_bytes(std::size_t bytes_received) {
    rx_accumulator_.insert(
        rx_accumulator_.end(),
        rx_buf_.begin(),
        rx_buf_.begin() + bytes_received
    );

    while (true) {
        std::uint16_t id = 0;
        std::vector<std::uint8_t> payload;

        if (!try_extract_packet(id, payload)) {
            break;
        }

        if (id == ENCODER_ANGLES_ID) {
            const auto angles = parse_encoders_payload(payload);

            if (encoder_callback_) {
                encoder_callback_(angles, serial_status::OK);
            }
        }
    }
}

bool GripperInterfaceDriver::try_extract_packet(
    std::uint16_t& id,
    std::vector<std::uint8_t>& payload
) {
    // Minimum packet:
    // SOF + ID_L + ID_H + LEN + CHECKSUM
    constexpr std::size_t minimum_packet_size = 5;

    while (!rx_accumulator_.empty() && rx_accumulator_[0] != SOF) {
        rx_accumulator_.erase(rx_accumulator_.begin());
    }

    if (rx_accumulator_.size() < minimum_packet_size) {
        return false;
    }

    const std::uint8_t id_l = rx_accumulator_[1];
    const std::uint8_t id_h = rx_accumulator_[2];
    const std::uint8_t len  = rx_accumulator_[3];

    const std::size_t full_packet_size =
        1 + 2 + 1 + static_cast<std::size_t>(len) + 1;

    if (rx_accumulator_.size() < full_packet_size) {
        return false;
    }

    id = static_cast<std::uint16_t>(id_l) |
         (static_cast<std::uint16_t>(id_h) << 8);

    payload.assign(
        rx_accumulator_.begin() + 4,
        rx_accumulator_.begin() + 4 + len
    );

    const std::uint8_t received_checksum =
        rx_accumulator_[full_packet_size - 1];

    const std::uint8_t calculated_checksum =
        checksum_xor(id, payload.data(), payload.size());

    rx_accumulator_.erase(
        rx_accumulator_.begin(),
        rx_accumulator_.begin() + full_packet_size
    );

    if (received_checksum != calculated_checksum) {
        return false;
    }

    return true;
}

std::vector<double> GripperInterfaceDriver::parse_encoders_payload(
    const std::vector<std::uint8_t>& payload
) {
    constexpr std::size_t num_angles = 2;
    constexpr std::uint16_t invalid_reading = 0xFFFF;

    std::vector<double> encoder_angles;
    encoder_angles.reserve(num_angles);

    if (payload.size() < num_angles * 2) {
        return encoder_angles;
    }

    for (std::size_t i = 0; i < num_angles; ++i) {
        const std::uint8_t lsb = payload[2 * i];
        const std::uint8_t msb = payload[2 * i + 1];

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

double GripperInterfaceDriver::raw_angle_to_radians(std::uint16_t raw_angle) {
    constexpr double two_pi = 2.0 * M_PI;
    constexpr double max_raw = 65535.0;

    return static_cast<double>(raw_angle) * two_pi / max_raw;
}


serial_status GripperInterfaceDriver::send_pwm(
    const std::vector<std::uint16_t>& pwm_values) {
    constexpr std::size_t num_servos = 3;
    constexpr std::size_t data_size = num_servos * 2;

    if (pwm_values.size() != num_servos) {
        return serial_status::ERR_BAD_PACKET;
    }

    std::array<std::uint8_t, data_size> buf{};

    for (std::size_t i = 0; i < num_servos; ++i) {
        const std::uint16_t pwm = pwm_values[i];

        buf[2 * i] =
            static_cast<std::uint8_t>(pwm & 0xFF);

        buf[2 * i + 1] =
            static_cast<std::uint8_t>((pwm >> 8) & 0xFF);
    }

    return send_packet(GRIPPER_PWM_ID, buf.data(), buf.size());
}

serial_status GripperInterfaceDriver::stop_gripper() {
    std::uint8_t data = 0x00;
    return send_packet(GRIPPER_STOP_ID, &data, 1);
}

serial_status GripperInterfaceDriver::start_gripper() {
    std::uint8_t data = 0x02;
    return send_packet(GRIPPER_START_ID, &data, 1);
}

