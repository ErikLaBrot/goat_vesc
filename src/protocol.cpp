#include "goat_vesc/vesc_protocol.hpp"

#include <cstring>

namespace goat_vesc {

// ── Internal helpers ──────────────────────────────────────────────────────────

namespace {

std::uint16_t read_u16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

std::int16_t read_i16(const std::uint8_t* p) {
    return static_cast<std::int16_t>(read_u16(p));
}

std::uint32_t read_u32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) |
           (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) <<  8) |
            static_cast<std::uint32_t>(p[3]);
}

std::int32_t read_i32(const std::uint8_t* p) {
    return static_cast<std::int32_t>(read_u32(p));
}

float read_f32(const std::uint8_t* p) {
    const std::uint32_t bits = read_u32(p);
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

} // namespace

// ── Framing ───────────────────────────────────────────────────────────────────

std::uint16_t VescProtocol::crc16ccitt(const Payload& data) noexcept {
    std::uint16_t crc = 0;
    for (const auto byte : data) {
        crc ^= static_cast<std::uint16_t>(byte) << 8;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000U)
                ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1);
        }
    }
    return crc;
}

VescProtocol::Payload VescProtocol::frame(const Payload& payload) {
    const std::size_t len = payload.size();
    Payload out;
    out.reserve(kMaxFramedPacketBytes);

    if (len <= 255) {
        out.push_back(0x02);
        out.push_back(static_cast<std::uint8_t>(len));
    } else {
        out.push_back(0x03);
        out.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
        out.push_back(static_cast<std::uint8_t>(len & 0xFF));
    }

    out.insert(out.end(), payload.begin(), payload.end());

    const std::uint16_t crc = crc16ccitt(payload);
    out.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    out.push_back(0x03); // stop byte

    return out;
}

// ── Request builders ──────────────────────────────────────────────────────────

VescProtocol::Payload VescProtocol::build_fw_version_request() {
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
    }));
}

VescProtocol::Payload VescProtocol::build_get_values_request() {
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::GetValues),
    }));
}

VescProtocol::Payload VescProtocol::build_get_imu_data_request(std::uint16_t mask) {
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::GetImuData),
        static_cast<std::uint16_t>(mask),
    }));
}

VescProtocol::Payload VescProtocol::build_set_rpm_command(std::int32_t rpm) {
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::SetRpm),
        rpm,
    }));
}

VescProtocol::Payload VescProtocol::build_set_duty_command(float duty) {
    // VESC expects duty as int32 scaled by 100000
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::SetDuty),
        static_cast<std::int32_t>(duty * 100000.0f),
    }));
}

VescProtocol::Payload VescProtocol::build_set_current_command(float amps) {
    // VESC expects current as int32 scaled by 1000 (milliamps)
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::SetCurrent),
        static_cast<std::int32_t>(amps * 1000.0f),
    }));
}

VescProtocol::Payload VescProtocol::build_set_current_brake_command(float amps) {
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::SetCurrentBrake),
        static_cast<std::int32_t>(amps * 1000.0f),
    }));
}

VescProtocol::Payload VescProtocol::build_set_servo_pos_command(float position) {
    // VESC servo position is typically sent as position * 1000 in a signed 16-bit field.
    return frame(builder_.build_packet({
        static_cast<std::uint8_t>(VescPacketCommID::SetServoPos),
        static_cast<std::int16_t>(position * 1000.0f),
    }));
}

// ── Response parsers ──────────────────────────────────────────────────────────

std::optional<FwVersion> VescProtocol::parse_fw_version(const Payload& payload) {
    // [0] comm_id  [1] major  [2] minor  [3..] hw name, uuid, ...
    if (payload.size() < 3) {
        return std::nullopt;
    }
    if (payload[0] != static_cast<std::uint8_t>(VescPacketCommID::FwVersion)) {
        return std::nullopt;
    }
    return FwVersion{payload[1], payload[2]};
}

std::optional<VescMotorState> VescProtocol::parse_get_values(const Payload& payload) {
    // Layout after the comm_id byte (offset from p = payload.data() + 1):
    //   +0  ..+1  : temp_fet          int16 / 10.0
    //   +2  ..+3  : temp_motor        int16 / 10.0
    //   +4  ..+7  : current_motor     int32 / 100.0
    //   +8  ..+11 : current_in        int32 / 100.0
    //   +12 ..+15 : id                int32 / 100.0  (skip)
    //   +16 ..+19 : iq                int32 / 100.0  (skip)
    //   +20 ..+21 : duty_now          int16 / 1000.0
    //   +22 ..+25 : rpm               int32
    //   +26 ..+27 : v_in              int16 / 10.0
    //   +28 ..+31 : amp_hours         int32 / 10000.0  (skip)
    //   +32 ..+35 : amp_hours_charged int32 / 10000.0  (skip)
    //   +36 ..+39 : watt_hours        int32 / 10000.0  (skip)
    //   +40 ..+43 : watt_hours_chgd   int32 / 10000.0  (skip)
    //   +44 ..+47 : tachometer        int32
    //   +48 ..+51 : tachometer_abs    int32
    //   +52       : fault_code        uint8
    constexpr std::size_t kMinLen = 1 + 53; // comm_id + 53 data bytes
    if (payload.size() < kMinLen) {
        return std::nullopt;
    }
    if (payload[0] != static_cast<std::uint8_t>(VescPacketCommID::GetValues)) {
        return std::nullopt;
    }

    const auto* p = payload.data() + 1;

    VescMotorState s;
    s.temp_fet      = static_cast<float>(read_i16(p +  0)) / 10.0f;
    s.temp_motor    = static_cast<float>(read_i16(p +  2)) / 10.0f;
    s.current_motor = static_cast<float>(read_i32(p +  4)) / 100.0f;
    s.current_in    = static_cast<float>(read_i32(p +  8)) / 100.0f;
    // skip id (+12), iq (+16)
    s.duty_cycle    = static_cast<float>(read_i16(p + 20)) / 1000.0f;
    s.rpm           = static_cast<float>(read_i32(p + 22));
    s.vin           = static_cast<float>(read_i16(p + 26)) / 10.0f;
    // skip amp_hours (+28), amp_hours_charged (+32), watt_hours (+36), watt_hours_charged (+40)
    s.tachometer     = read_i32(p + 44);
    s.tachometer_abs = read_i32(p + 48);
    s.fault_code     = p[52];

    return s;
}

std::optional<VescIMUData> VescProtocol::parse_get_imu_data(const Payload& payload) {
    // Layout:
    //   [0]      comm_id  (GetImuData = 65)
    //   [1..2]   mask     uint16 — which fields follow
    //   [3..]    one 4-byte big-endian IEEE 754 float per set bit, in bit order:
    //              bit 0  Roll     bit 4  AccY    bit  8  GyroZ   bit 12  QuatW
    //              bit 1  Pitch    bit 5  AccZ    bit  9  MagX    bit 13  QuatX
    //              bit 2  Yaw      bit 6  GyroX   bit 10  MagY    bit 14  QuatY
    //              bit 3  AccX     bit 7  GyroY   bit 11  MagZ    bit 15  QuatZ
    if (payload.size() < 3) return std::nullopt;
    if (payload[0] != static_cast<std::uint8_t>(VescPacketCommID::GetImuData)) return std::nullopt;

    const std::uint16_t mask = read_u16(payload.data() + 1);
    const auto field_count   = static_cast<std::size_t>(__builtin_popcount(mask));

    if (payload.size() < 3 + field_count * 4) return std::nullopt;

    const auto* p = payload.data() + 3;
    VescIMUData d;

    auto next = [&]() -> float {
        const float v = read_f32(p);
        p += 4;
        return v;
    };

    if (mask & static_cast<std::uint16_t>(VescImuMask::Roll))   d.roll   = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::Pitch))  d.pitch  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::Yaw))    d.yaw    = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::AccX))   d.acc_x  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::AccY))   d.acc_y  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::AccZ))   d.acc_z  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::GyroX))  d.gyro_x = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::GyroY))  d.gyro_y = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::GyroZ))  d.gyro_z = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::MagX))   d.mag_x  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::MagY))   d.mag_y  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::MagZ))   d.mag_z  = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::QuatW))  d.quat_w = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::QuatX))  d.quat_x = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::QuatY))  d.quat_y = next();
    if (mask & static_cast<std::uint16_t>(VescImuMask::QuatZ))  d.quat_z = next();

    return d;
}

} // namespace goat_vesc
