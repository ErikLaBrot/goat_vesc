/*
* We build and parse packets here to talk to and from the vesc
*/

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "goat_vesc/types.hpp"
#include "goat_vesc/protocol_ids.hpp"
#include "goat_vesc/packet_builder.hpp"

namespace goat_vesc {

class VescProtocol {
public:
    using Payload = std::vector<std::uint8_t>;

    // ── Request builders ──────────────────────────────────────────────────────
    // Each returns a complete, framed packet ready to write to the serial port.

    Payload build_fw_version_request();
    Payload build_get_values_request();
    Payload build_get_imu_data_request(std::uint16_t mask = DefaultVescImuMask);
    Payload build_set_rpm_command(std::int32_t rpm);
    Payload build_set_duty_command(float duty);           // -1.0 to 1.0
    Payload build_set_current_command(float amps);
    Payload build_set_current_brake_command(float amps);
    Payload build_set_servo_pos_command(float position);

    // ── Response parsers ──────────────────────────────────────────────────────
    // Each takes a raw payload (stripped of framing/CRC by VescPacketParser)
    // and returns the typed result, or nullopt if the payload is malformed.

    std::optional<FwVersion>      parse_fw_version(const Payload& payload);
    std::optional<VescMotorState> parse_get_values(const Payload& payload);
    std::optional<VescIMUData>    parse_get_imu_data(const Payload& payload);

private:
    // Pre-allocated once from kMaxPayloadBytes; reused across all build calls.
    VescPacketBuilder builder_{kMaxPayloadBytes};

    // Wraps a payload in the VESC framing: [start | len... | payload | crc | stop]
    static Payload frame(const Payload& payload);
    static std::uint16_t crc16ccitt(const Payload& data) noexcept;
};

} // namespace goat_vesc
