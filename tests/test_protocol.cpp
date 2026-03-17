#include "test_support.hpp"

#include "goat_vesc/packet_parser.hpp"
#include "goat_vesc/protocol_ids.hpp"
#include "goat_vesc/vesc_protocol.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace {

using namespace goat_vesc;
using namespace test_support;

constexpr float kFloatTolerance = 1.0e-6f;

void expect_near(float actual, float expected) {
    assert(std::fabs(actual - expected) <= kFloatTolerance);
}

std::vector<std::uint8_t> make_values_payload() {
    std::vector<std::uint8_t> payload;
    append_u8(payload, static_cast<std::uint8_t>(VescPacketCommID::GetValues));
    append_i16(payload, 325);
    append_i16(payload, -15);
    append_i32(payload, 1234);
    append_i32(payload, -890);
    append_i32(payload, 111);
    append_i32(payload, -222);
    append_i16(payload, -321);
    append_i32(payload, -12345);
    append_i16(payload, 523);
    append_i32(payload, 1000);
    append_i32(payload, -2000);
    append_i32(payload, 3000);
    append_i32(payload, -4000);
    append_i32(payload, -77);
    append_i32(payload, 88);
    append_u8(payload, 7);
    return payload;
}

std::vector<std::uint8_t> make_imu_payload(
    std::uint16_t mask, std::initializer_list<float> values) {

    std::vector<std::uint8_t> payload;
    append_u8(payload, static_cast<std::uint8_t>(VescPacketCommID::GetImuData));
    append_u16(payload, mask);
    for (const auto value : values) {
        append_f32(payload, value);
    }
    return payload;
}

void test_build_requests_exact_bytes() {
    VescProtocol protocol;

    assert((protocol.build_fw_version_request() ==
        std::vector<std::uint8_t>{0x02, 0x01, 0x00, 0x00, 0x00, 0x03}));

    assert((protocol.build_get_values_request() ==
        std::vector<std::uint8_t>{0x02, 0x01, 0x04, 0x40, 0x84, 0x03}));

    assert((protocol.build_get_imu_data_request() ==
        std::vector<std::uint8_t>{0x02, 0x03, 0x41, 0xFF, 0xFF, 0x37, 0x92, 0x03}));

    assert((protocol.build_get_imu_data_request(0x1234U) ==
        std::vector<std::uint8_t>{0x02, 0x03, 0x41, 0x12, 0x34, 0x39, 0x5B, 0x03}));
}

void test_build_control_commands_exact_bytes() {
    VescProtocol protocol;

    assert((protocol.build_set_rpm_command(1000) ==
        std::vector<std::uint8_t>{0x02, 0x05, 0x08, 0x00, 0x00, 0x03, 0xE8, 0x2B, 0x58, 0x03}));

    assert((protocol.build_set_duty_command(0.2f) ==
        std::vector<std::uint8_t>{0x02, 0x05, 0x05, 0x00, 0x00, 0x4E, 0x20, 0x29, 0xF6, 0x03}));

    assert((protocol.build_set_current_command(1.5f) ==
        std::vector<std::uint8_t>{0x02, 0x05, 0x06, 0x00, 0x00, 0x05, 0xDC, 0x38, 0x81, 0x03}));

    assert((protocol.build_set_current_brake_command(-1.5f) ==
        std::vector<std::uint8_t>{0x02, 0x05, 0x07, 0xFF, 0xFF, 0xFA, 0x24, 0x7B, 0xF8, 0x03}));

    assert((protocol.build_set_servo_pos_command(0.5f) ==
        std::vector<std::uint8_t>{0x02, 0x03, 0x0C, 0x01, 0xF4, 0xE9, 0xCB, 0x03}));
}

void test_parse_fw_version() {
    VescProtocol protocol;

    const auto parsed = protocol.parse_fw_version({
        static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
        6,
        5,
    });
    assert(parsed.has_value());
    assert(parsed->major == 6);
    assert(parsed->minor == 5);

    assert(!protocol.parse_fw_version({
        static_cast<std::uint8_t>(VescPacketCommID::GetValues),
        6,
        5,
    }).has_value());
    assert(!protocol.parse_fw_version({
        static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
        6,
    }).has_value());
}

void test_parse_get_values() {
    VescProtocol protocol;

    const auto parsed = protocol.parse_get_values(make_values_payload());
    assert(parsed.has_value());
    expect_near(parsed->temp_fet, 32.5f);
    expect_near(parsed->temp_motor, -1.5f);
    expect_near(parsed->current_motor, 12.34f);
    expect_near(parsed->current_in, -8.90f);
    expect_near(parsed->duty_cycle, -0.321f);
    expect_near(parsed->rpm, -12345.0f);
    expect_near(parsed->vin, 52.3f);
    assert(parsed->tachometer == -77);
    assert(parsed->tachometer_abs == 88);
    assert(parsed->fault_code == 7);

    assert(!protocol.parse_get_values({
        static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
    }).has_value());

    auto truncated = make_values_payload();
    truncated.pop_back();
    assert(!protocol.parse_get_values(truncated).has_value());
}

void test_parse_get_imu_data_full_mask() {
    VescProtocol protocol;
    const auto payload = make_imu_payload(
        DefaultVescImuMask,
        {0.25f, 0.50f, 0.75f, 1.00f, 1.25f, 1.50f, 1.75f, 2.00f,
         2.25f, 2.50f, 2.75f, 3.00f, 0.10f, 0.20f, 0.30f, 0.40f});

    const auto parsed = protocol.parse_get_imu_data(payload);
    assert(parsed.has_value());
    expect_near(parsed->roll, 0.25f);
    expect_near(parsed->pitch, 0.50f);
    expect_near(parsed->yaw, 0.75f);
    expect_near(parsed->acc_x, 1.00f);
    expect_near(parsed->acc_y, 1.25f);
    expect_near(parsed->acc_z, 1.50f);
    expect_near(parsed->gyro_x, 1.75f);
    expect_near(parsed->gyro_y, 2.00f);
    expect_near(parsed->gyro_z, 2.25f);
    expect_near(parsed->mag_x, 2.50f);
    expect_near(parsed->mag_y, 2.75f);
    expect_near(parsed->mag_z, 3.00f);
    expect_near(parsed->quat_w, 0.10f);
    expect_near(parsed->quat_x, 0.20f);
    expect_near(parsed->quat_y, 0.30f);
    expect_near(parsed->quat_z, 0.40f);
}

void test_parse_get_imu_data_sparse_mask_and_rejections() {
    VescProtocol protocol;
    constexpr std::uint16_t kMask =
        static_cast<std::uint16_t>(VescImuMask::Roll) |
        static_cast<std::uint16_t>(VescImuMask::AccY) |
        static_cast<std::uint16_t>(VescImuMask::QuatZ);

    const auto parsed = protocol.parse_get_imu_data(make_imu_payload(kMask, {1.0f, -2.0f, 0.5f}));
    assert(parsed.has_value());
    expect_near(parsed->roll, 1.0f);
    expect_near(parsed->acc_y, -2.0f);
    expect_near(parsed->quat_z, 0.5f);
    expect_near(parsed->pitch, 0.0f);
    expect_near(parsed->quat_w, 1.0f);

    assert(!protocol.parse_get_imu_data({
        static_cast<std::uint8_t>(VescPacketCommID::GetValues),
        0x00,
        0x00,
    }).has_value());

    assert(!protocol.parse_get_imu_data({
        static_cast<std::uint8_t>(VescPacketCommID::GetImuData),
        0x00,
    }).has_value());

    assert(!protocol.parse_get_imu_data(make_imu_payload(kMask, {1.0f, -2.0f})).has_value());
}

void test_packet_parser_round_trip_and_resync() {
    VescProtocol protocol;
    VescPacketParser parser;

    auto payloads = parser.feed_bytes(protocol.build_get_imu_data_request());
    assert(payloads.size() == 1);
    const std::vector<std::uint8_t> expected_imu_request{0x41, 0xFF, 0xFF};
    assert(payloads.front() == expected_imu_request);

    auto invalid = protocol.build_get_values_request();
    invalid[3] ^= 0x7F;
    const auto valid = protocol.build_fw_version_request();

    assert(parser.feed_bytes(invalid).empty());
    payloads = parser.feed_bytes(valid);
    assert(payloads.size() == 1);
    const std::vector<std::uint8_t> expected_fw_payload{0x00};
    assert(payloads.front() == expected_fw_payload);
}

void test_packet_parser_frame_boundaries() {
    VescPacketParser parser;

    const std::vector<std::uint8_t> short_payload(255, 0x11);
    auto payloads = parser.feed_bytes(frame_payload(short_payload));
    assert(payloads.size() == 1);
    assert(payloads.front() == short_payload);

    parser.reset();

    const std::vector<std::uint8_t> medium_payload(256, 0x22);
    payloads = parser.feed_bytes(frame_payload(medium_payload));
    assert(payloads.size() == 1);
    assert(payloads.front() == medium_payload);
}

void test_packet_parser_rejects_invalid_frames() {
    VescProtocol protocol;
    VescPacketParser parser;

    const auto valid = protocol.build_fw_version_request();

    std::vector<std::uint8_t> zero_length_then_valid{0x02, 0x00};
    zero_length_then_valid.insert(zero_length_then_valid.end(), valid.begin(), valid.end());
    auto payloads = parser.feed_bytes(zero_length_then_valid);
    assert(payloads.size() == 1);
    const std::vector<std::uint8_t> expected_fw_payload{0x00};
    assert(payloads.front() == expected_fw_payload);

    parser.reset();

    std::vector<std::uint8_t> unsupported_long_then_valid{0x04, 0x00, 0x00, 0x01};
    unsupported_long_then_valid.insert(unsupported_long_then_valid.end(), valid.begin(), valid.end());
    payloads = parser.feed_bytes(unsupported_long_then_valid);
    assert(payloads.size() == 1);
    assert(payloads.front() == expected_fw_payload);

    parser.reset();

    auto truncated = valid;
    truncated.pop_back();
    assert(parser.feed_bytes(truncated).empty());
}

} // namespace

int main() {
    test_build_requests_exact_bytes();
    test_build_control_commands_exact_bytes();
    test_parse_fw_version();
    test_parse_get_values();
    test_parse_get_imu_data_full_mask();
    test_parse_get_imu_data_sparse_mask_and_rejections();
    test_packet_parser_round_trip_and_resync();
    test_packet_parser_frame_boundaries();
    test_packet_parser_rejects_invalid_frames();
    return 0;
}
