#include "test_support.hpp"

#include "goat_vesc/packet_parser.hpp"
#include "goat_vesc/protocol_ids.hpp"
#include "goat_vesc/vesc_protocol.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <vector>

namespace {

using namespace goat_vesc;
using namespace test_support;

constexpr float kFloatTolerance = 1.0e-6f;

void expect_near(float actual, float expected) {
  assert(std::fabs(actual - expected) <= kFloatTolerance);
}

std::vector<std::uint8_t> bytes(std::initializer_list<std::uint8_t> values) {
  return {values};
}

std::vector<std::uint8_t> concat_bytes(std::initializer_list<std::vector<std::uint8_t>> chunks) {
  const auto total_size = std::accumulate(
      chunks.begin(), chunks.end(), std::size_t{0},
      [](std::size_t size, const std::vector<std::uint8_t>& chunk) { return size + chunk.size(); });

  std::vector<std::uint8_t> combined;
  combined.reserve(total_size);
  for (const auto& chunk : chunks) {
    combined.insert(combined.end(), chunk.begin(), chunk.end());
  }
  return combined;
}

std::vector<std::uint8_t> make_long16_header(std::uint16_t payload_len) {
  return {
      0x03,
      static_cast<std::uint8_t>((payload_len >> 8) & 0xFF),
      static_cast<std::uint8_t>(payload_len & 0xFF),
  };
}

std::vector<std::uint8_t> make_long24_header(std::uint32_t payload_len) {
  return {
      0x04,
      static_cast<std::uint8_t>((payload_len >> 16) & 0xFF),
      static_cast<std::uint8_t>((payload_len >> 8) & 0xFF),
      static_cast<std::uint8_t>(payload_len & 0xFF),
  };
}

std::vector<std::uint8_t> make_values_payload() {
  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(VescPacketCommID::GetValues));
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
  payload.push_back(7);
  return payload;
}

std::vector<std::uint8_t> make_imu_payload(std::uint16_t mask,
                                           std::initializer_list<float> values) {

  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(VescPacketCommID::GetImuData));
  append_u16(payload, mask);
  for (const auto value : values) {
    append_f32(payload, value);
  }
  return payload;
}

void test_build_requests_exact_bytes() {
  assert((VescProtocol::build_fw_version_request() ==
          std::vector<std::uint8_t>{0x02, 0x01, 0x00, 0x00, 0x00, 0x03}));

  assert((VescProtocol::build_get_values_request() ==
          std::vector<std::uint8_t>{0x02, 0x01, 0x04, 0x40, 0x84, 0x03}));

  assert((VescProtocol::build_get_imu_data_request() ==
          std::vector<std::uint8_t>{0x02, 0x03, 0x41, 0xFF, 0xFF, 0x37, 0x92, 0x03}));
}

void test_build_management_requests_exact_bytes() {
  assert(VescProtocol::build_get_motor_config_request() ==
         frame_payload({static_cast<std::uint8_t>(VescPacketCommID::GetMotorConfig)}));
  assert(VescProtocol::build_get_app_config_request() ==
         frame_payload({static_cast<std::uint8_t>(VescPacketCommID::GetAppConfig)}));

  const MotorConfigImage motor{{0x11, 0x22, 0x33, 0x44, 0x55}};
  assert(VescProtocol::build_set_motor_config_request(motor) ==
         frame_payload({static_cast<std::uint8_t>(VescPacketCommID::SetMotorConfig),
                        0x11, 0x22, 0x33, 0x44, 0x55}));

  const AppConfigImage app{{0xAA, 0xBB, 0xCC, 0xDD, 0x01}};
  assert(VescProtocol::build_set_app_config_request(app, AppConfigStorage::Persistent) ==
         frame_payload({static_cast<std::uint8_t>(VescPacketCommID::SetAppConfig),
                        0xAA, 0xBB, 0xCC, 0xDD, 0x01}));
  assert(VescProtocol::build_set_app_config_request(app, AppConfigStorage::Volatile) ==
         frame_payload({static_cast<std::uint8_t>(VescPacketCommID::SetAppConfigNoStore),
                        0xAA, 0xBB, 0xCC, 0xDD, 0x01}));

  std::vector<std::uint8_t> read_lisp{
      static_cast<std::uint8_t>(VescPacketCommID::LispReadCode)};
  append_i32(read_lisp, 400);
  append_i32(read_lisp, 10);
  assert(VescProtocol::build_lisp_read_request(400, 10) == frame_payload(read_lisp));

  std::vector<std::uint8_t> erase_lisp{
      static_cast<std::uint8_t>(VescPacketCommID::LispEraseCode)};
  append_i32(erase_lisp, 1234);
  assert(VescProtocol::build_lisp_erase_request(1234) == frame_payload(erase_lisp));

  std::vector<std::uint8_t> write_lisp{
      static_cast<std::uint8_t>(VescPacketCommID::LispWriteCode)};
  append_i32(write_lisp, 384);
  write_lisp.insert(write_lisp.end(), {0x10, 0x20, 0x30});
  assert(VescProtocol::build_lisp_write_request({0x10, 0x20, 0x30}, 384) ==
         frame_payload(write_lisp));

  assert(VescProtocol::build_lisp_set_running_request(true) ==
         frame_payload({static_cast<std::uint8_t>(VescPacketCommID::LispSetRunning), 1}));

  FocCalibrationParameters calibration{50.0f, -12.5f, 30.0f, 1200.0f, 3500.0f};
  std::vector<std::uint8_t> calibrate{
      static_cast<std::uint8_t>(VescPacketCommID::DetectApplyAllFoc), 0};
  append_i32(calibrate, 50000);
  append_i32(calibrate, -12500);
  append_i32(calibrate, 30000);
  append_i32(calibrate, 1200000);
  append_i32(calibrate, 3500000);
  assert(VescProtocol::build_foc_calibration_request(calibration) ==
         frame_payload(calibrate));

  std::vector<std::uint8_t> disable{
      static_cast<std::uint8_t>(VescPacketCommID::AppDisableOutput), 0};
  append_i32(disable, 185000);
  assert(VescProtocol::build_app_disable_output_command(std::chrono::milliseconds(185000)) ==
         frame_payload(disable));

  const LispCodeImage code{{'(', ')', '\0'}};
  std::vector<std::uint8_t> crc_input{0, 0, '(', ')', '\0'};
  std::vector<std::uint8_t> packed;
  append_i32(packed, 3);
  append_u16(packed, crc16ccitt(crc_input));
  packed.insert(packed.end(), crc_input.begin(), crc_input.end());
  assert(VescProtocol::pack_lisp_code(code) == packed);
}

void test_management_protocol_validation() {
  assert(VescProtocol::build_set_motor_config_request(MotorConfigImage{{1, 2, 3}}).empty());
  assert(VescProtocol::build_set_app_config_request(AppConfigImage{{1, 2, 3}},
                                                    AppConfigStorage::Persistent)
             .empty());
  assert(VescProtocol::build_set_motor_config_request(
             MotorConfigImage{std::vector<std::uint8_t>(kMaxPayloadBytes, 0)})
             .empty());
  assert(VescProtocol::build_lisp_read_request(0, 0).empty());
  assert(VescProtocol::build_lisp_read_request(kMaxPayloadBytes - 9, 0).empty());
  assert(VescProtocol::build_lisp_write_request({}, 0).empty());
  assert(VescProtocol::build_lisp_write_request(
             std::vector<std::uint8_t>(kMaxPayloadBytes - 4, 0), 0)
             .empty());
  assert(VescProtocol::pack_lisp_code(LispCodeImage{}).empty());
  assert(VescProtocol::pack_lisp_code(
             LispCodeImage{std::vector<std::uint8_t>(kMaxLispCodeBytes + 1, 0)})
             .empty());
  assert(VescProtocol::build_set_app_config_request(
             AppConfigImage{{1, 2, 3, 4}}, static_cast<AppConfigStorage>(99))
             .empty());
  assert(VescProtocol::build_foc_calibration_request({}).empty());
  assert(VescProtocol::build_foc_calibration_request({50.0f, 1.0f}).empty());
  assert(VescProtocol::build_foc_calibration_request({50.0f, 0.0f, -1.0f}).empty());
  assert(VescProtocol::build_foc_calibration_request({100000.0f}).empty());
  assert(VescProtocol::build_foc_calibration_request({50.0f, -10000.0f}).empty());
  assert(VescProtocol::build_foc_calibration_request({50.0f, 0.0f, 10000.0f}).empty());
  assert(
      VescProtocol::build_foc_calibration_request({50.0f, 0.0f, 0.0f, 1000000.0f})
          .empty());
  assert(VescProtocol::build_foc_calibration_request(
             {50.0f, 0.0f, 0.0f, 0.0f, 1000000.0f})
             .empty());
  assert(VescProtocol::build_foc_calibration_request(
             {50.0f, 0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f})
             .empty());
  assert(VescProtocol::build_app_disable_output_command(std::chrono::milliseconds(-1)).empty());

  const auto motor =
      VescProtocol::parse_motor_config({static_cast<std::uint8_t>(
                                            VescPacketCommID::GetMotorConfig),
                                        1, 2, 3, 4, 5});
  assert(motor && motor->bytes == bytes({1, 2, 3, 4, 5}));
  assert(!VescProtocol::parse_motor_config(
              {static_cast<std::uint8_t>(VescPacketCommID::GetMotorConfig), 1, 2, 3})
              .has_value());

  assert(VescProtocol::parse_config_ack(
      {static_cast<std::uint8_t>(VescPacketCommID::SetMotorConfig)},
      VescPacketCommID::SetMotorConfig));
  assert(!VescProtocol::parse_config_ack(
      {static_cast<std::uint8_t>(VescPacketCommID::SetMotorConfig), 1},
      VescPacketCommID::SetMotorConfig));

  std::vector<std::uint8_t> read_reply{
      static_cast<std::uint8_t>(VescPacketCommID::LispReadCode)};
  append_i32(read_reply, 12);
  append_i32(read_reply, 4);
  read_reply.insert(read_reply.end(), {0xA0, 0xA1, 0xA2});
  std::uint32_t total = 0;
  std::uint32_t offset = 0;
  const auto chunk = VescProtocol::parse_lisp_read_reply(read_reply, total, offset);
  assert(chunk == std::optional<std::vector<std::uint8_t>>({0xA0, 0xA1, 0xA2}));
  assert(total == 12);
  assert(offset == 4);

  read_reply[4] = 2;
  assert(!VescProtocol::parse_lisp_read_reply(read_reply, total, offset).has_value());

  std::vector<std::uint8_t> write_ack{
      static_cast<std::uint8_t>(VescPacketCommID::LispWriteCode), 1};
  append_i32(write_ack, 384);
  assert(VescProtocol::parse_lisp_write_ack(write_ack, 384));
  assert(!VescProtocol::parse_lisp_write_ack(write_ack, 0));
  write_ack[1] = 0;
  assert(!VescProtocol::parse_lisp_write_ack(write_ack, 384));
  write_ack[1] = 2;
  assert(!VescProtocol::parse_lisp_write_ack(write_ack, 384));

  assert(VescProtocol::parse_bool_ack(
      {static_cast<std::uint8_t>(VescPacketCommID::LispEraseCode), 1},
      VescPacketCommID::LispEraseCode));
  assert(!VescProtocol::parse_bool_ack(
      {static_cast<std::uint8_t>(VescPacketCommID::LispEraseCode)},
      VescPacketCommID::LispEraseCode));

  const auto foc_ok = VescProtocol::parse_foc_calibration_reply(
      {static_cast<std::uint8_t>(VescPacketCommID::DetectApplyAllFoc), 0, 2});
  assert(foc_ok && *foc_ok == 2);
  const auto foc_failed = VescProtocol::parse_foc_calibration_reply(
      {static_cast<std::uint8_t>(VescPacketCommID::DetectApplyAllFoc), 0xFF, 0xF6});
  assert(foc_failed && *foc_failed == -10);
  assert(!VescProtocol::parse_foc_calibration_reply(
              {static_cast<std::uint8_t>(VescPacketCommID::DetectApplyAllFoc), 0})
              .has_value());

  MotorConfigImage long_config{std::vector<std::uint8_t>(300, 0x55)};
  const auto framed = VescProtocol::build_set_motor_config_request(long_config);
  assert(!framed.empty() && framed.front() == 0x03);
  VescPacketParser parser;
  const auto payloads = parser.feed_bytes(framed);
  assert(payloads.size() == 1);
  assert(payloads.front().size() == long_config.bytes.size() + 1);
}

void test_build_control_commands_exact_bytes() {
  assert((VescProtocol::build_set_rpm_command(1000) ==
          std::vector<std::uint8_t>{0x02, 0x05, 0x08, 0x00, 0x00, 0x03, 0xE8, 0x2B, 0x58, 0x03}));

  assert((VescProtocol::build_set_duty_command(0.2f) ==
          std::vector<std::uint8_t>{0x02, 0x05, 0x05, 0x00, 0x00, 0x4E, 0x20, 0x29, 0xF6, 0x03}));

  assert((VescProtocol::build_set_current_command(1.5f) ==
          std::vector<std::uint8_t>{0x02, 0x05, 0x06, 0x00, 0x00, 0x05, 0xDC, 0x38, 0x81, 0x03}));

  assert((VescProtocol::build_set_current_brake_command(-1.5f) ==
          std::vector<std::uint8_t>{0x02, 0x05, 0x07, 0xFF, 0xFF, 0xFA, 0x24, 0x7B, 0xF8, 0x03}));

  assert((VescProtocol::build_set_servo_pos_command(0.5f) ==
          std::vector<std::uint8_t>{0x02, 0x03, 0x0C, 0x01, 0xF4, 0xE9, 0xCB, 0x03}));
}

void test_control_builders_reject_invalid_floats() {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const float largest = std::numeric_limits<float>::max();

  assert(VescProtocol::build_set_duty_command(nan).empty());
  assert(VescProtocol::build_set_duty_command(1.01f).empty());
  assert(VescProtocol::build_set_current_command(infinity).empty());
  assert(VescProtocol::build_set_current_brake_command(largest).empty());
  assert(VescProtocol::build_set_servo_pos_command(-0.01f).empty());
  assert(VescProtocol::build_set_servo_pos_command(nan).empty());
}

void test_parse_fw_version() {
  const auto parsed = VescProtocol::parse_fw_version({
      static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
      6,
      5,
  });
  assert(parsed.has_value());
  assert(parsed->major == 6);
  assert(parsed->minor == 5);

  assert(!VescProtocol::parse_fw_version({
              static_cast<std::uint8_t>(VescPacketCommID::GetValues),
              6,
              5,
          })
              .has_value());
  assert(!VescProtocol::parse_fw_version({
              static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
              6,
          })
              .has_value());
}

void test_parse_get_values() {
  const auto parsed = VescProtocol::parse_get_values(make_values_payload());
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

  assert(!VescProtocol::parse_get_values({
              static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
          })
              .has_value());

  auto truncated = make_values_payload();
  truncated.pop_back();
  assert(!VescProtocol::parse_get_values(truncated).has_value());
}

void test_parse_get_imu_data_full_mask() {
  const auto payload = make_imu_payload(0xFFFFU,
                                        {0.25f, 0.50f, 0.75f, 1.00f, 1.25f, 1.50f, 1.75f, 2.00f,
                                         2.25f, 2.50f, 2.75f, 3.00f, 0.10f, 0.20f, 0.30f, 0.40f});

  const auto parsed = VescProtocol::parse_get_imu_data(payload);
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
  constexpr std::uint16_t kMask = (1U << 0U) | (1U << 4U) | (1U << 15U);

  const auto parsed =
      VescProtocol::parse_get_imu_data(make_imu_payload(kMask, {1.0f, -2.0f, 0.5f}));
  assert(parsed.has_value());
  expect_near(parsed->roll, 1.0f);
  expect_near(parsed->acc_y, -2.0f);
  expect_near(parsed->quat_z, 0.5f);
  expect_near(parsed->pitch, 0.0f);
  expect_near(parsed->quat_w, 1.0f);

  assert(!VescProtocol::parse_get_imu_data({
              static_cast<std::uint8_t>(VescPacketCommID::GetValues),
              0x00,
              0x00,
          })
              .has_value());

  assert(!VescProtocol::parse_get_imu_data({
              static_cast<std::uint8_t>(VescPacketCommID::GetImuData),
              0x00,
          })
              .has_value());

  assert(!VescProtocol::parse_get_imu_data(make_imu_payload(kMask, {1.0f, -2.0f})).has_value());
}

void test_packet_parser_round_trip_and_resync() {
  VescPacketParser parser;

  auto payloads = parser.feed_bytes(VescProtocol::build_get_imu_data_request());
  assert(payloads.size() == 1);
  const std::vector<std::uint8_t> expected_imu_request{0x41, 0xFF, 0xFF};
  assert(payloads.front() == expected_imu_request);

  auto invalid = VescProtocol::build_get_values_request();
  invalid[3] ^= 0x7F;
  const auto valid = VescProtocol::build_fw_version_request();

  assert(parser.feed_bytes(invalid).empty());
  payloads = parser.feed_bytes(valid);
  assert(payloads.size() == 1);
  const std::vector<std::uint8_t> expected_fw_payload{0x00};
  assert(payloads.front() == expected_fw_payload);
}

void test_packet_parser_short_frame_incremental_delivery() {
  VescPacketParser parser;
  const auto expected_payload = bytes({0x11, 0x22, 0x33});
  const auto framed = frame_payload(expected_payload);

  for (std::size_t i = 0; i + 1 < framed.size(); ++i) {
    assert(!parser.feed_byte(framed[i]).has_value());
  }

  const auto parsed = parser.feed_byte(framed.back());
  assert(parsed.has_value());
  assert(*parsed == expected_payload);
}

void test_packet_parser_waits_for_outer_frame_with_nested_frame_bytes() {
  const auto nested = VescProtocol::build_fw_version_request();
  auto outer_payload = bytes({0x42});
  outer_payload.insert(outer_payload.end(), nested.begin(), nested.end());
  outer_payload.push_back(0x43);
  const auto outer = frame_payload(outer_payload);

  const auto expect_outer = [&](VescPacketParser& parser) {
    const std::vector<std::uint8_t> without_outer_trailer(outer.begin(), outer.end() - 3);
    assert(parser.feed_bytes(without_outer_trailer).empty());

    const std::vector<std::uint8_t> outer_trailer(outer.end() - 3, outer.end());
    const auto payloads = parser.feed_bytes(outer_trailer);
    assert(payloads.size() == 1);
    assert(payloads.front() == outer_payload);
  };

  VescPacketParser fresh_parser;
  expect_outer(fresh_parser);

  VescPacketParser parser_after_error;
  auto invalid = frame_payload(bytes({0xA0}));
  invalid[invalid.size() - 2] ^= 0x01;
  assert(parser_after_error.feed_bytes(invalid).empty());
  expect_outer(parser_after_error);
}

void test_packet_parser_emits_multiple_frames_from_one_burst() {
  VescPacketParser parser;
  const auto first_payload = bytes({0x01});
  const auto second_payload = bytes({0x02, 0x03});

  const auto payloads = parser.feed_bytes(
      concat_bytes({frame_payload(first_payload), frame_payload(second_payload)}));

  assert(payloads.size() == 2);
  assert(payloads[0] == first_payload);
  assert(payloads[1] == second_payload);
}

void test_packet_parser_resyncs_after_garbage_prefix() {
  VescPacketParser parser;
  const auto expected_payload = bytes({0x44, 0x55});

  const auto payloads = parser.feed_bytes(
      concat_bytes({bytes({0x00, 0x99, 0x01, 0x7F}), frame_payload(expected_payload)}));

  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);
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

  parser.reset();

  const std::vector<std::uint8_t> max_payload(kMaxPayloadBytes, 0x33);
  payloads = parser.feed_bytes(frame_payload(max_payload));
  assert(payloads.size() == 1);
  assert(payloads.front() == max_payload);
}

void test_packet_parser_resyncs_after_bad_crc() {
  VescPacketParser parser;
  auto invalid = frame_payload(bytes({0xA0, 0xA1}));
  invalid[invalid.size() - 2] ^= 0x01;

  const auto expected_payload = bytes({0xB0});
  const auto payloads = parser.feed_bytes(concat_bytes({invalid, frame_payload(expected_payload)}));

  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);
}

void test_packet_parser_resyncs_after_bad_stop_byte() {
  VescPacketParser parser;
  auto invalid = frame_payload(bytes({0xC0, 0xC1}));
  invalid.back() = 0x00;

  const auto expected_payload = bytes({0xD0});
  const auto payloads = parser.feed_bytes(concat_bytes({invalid, frame_payload(expected_payload)}));

  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);
}

void test_packet_parser_resyncs_after_corrupt_length() {
  VescPacketParser parser;
  const auto valid = VescProtocol::build_fw_version_request();
  // Length bytes are not escaped, so a corrupt length can consume later frames
  // within one declared candidate. A following frame restores synchronization.
  const auto payloads =
      parser.feed_bytes(concat_bytes({bytes({0x02, 0x03, 0xAA}), valid, valid}));

  assert(payloads.size() == 1);
  assert(payloads.front() == bytes({static_cast<std::uint8_t>(VescPacketCommID::FwVersion)}));
}

void test_packet_parser_reset_clears_partial_frame_state() {
  VescPacketParser parser;
  const auto expected_payload = bytes({0x21, 0x22});
  const auto framed = frame_payload(expected_payload);

  const std::vector<std::uint8_t> partial(framed.begin(), framed.end() - 1);
  assert(parser.feed_bytes(partial).empty());

  parser.reset();

  assert(!parser.feed_byte(framed.back()).has_value());

  parser.reset();

  const auto payloads = parser.feed_bytes(framed);
  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);
}

void test_packet_parser_rejects_invalid_medium_frame_lengths() {
  VescPacketParser parser;
  const auto expected_payload = bytes({0xE0});
  // Keep the recovery probe on the short-frame path so any decoded payload must
  // come from a clean resync after the invalid medium-length header.
  const auto valid_short_frame = frame_payload(expected_payload);
  assert(valid_short_frame.front() == 0x02);

  auto payloads = parser.feed_bytes(make_long16_header(255));
  assert(payloads.empty());

  payloads = parser.feed_bytes(valid_short_frame);
  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);

  parser.reset();

  payloads = parser.feed_bytes(make_long16_header(254));
  assert(payloads.empty());

  payloads = parser.feed_bytes(valid_short_frame);
  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);

  parser.reset();

  payloads =
      parser.feed_bytes(make_long16_header(static_cast<std::uint16_t>(kMaxPayloadBytes + 1U)));
  assert(payloads.empty());

  payloads = parser.feed_bytes(concat_bytes({valid_short_frame, valid_short_frame}));
  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);
}

void test_packet_parser_resyncs_after_unsupported_24bit_sequences() {
  VescPacketParser parser;
  const auto expected_payload = bytes({0xF0});
  const auto valid_short_frame = frame_payload(expected_payload);
  assert(valid_short_frame.front() == 0x02);

  auto payloads = parser.feed_bytes(bytes({0x04}));
  assert(payloads.empty());

  payloads = parser.feed_bytes(valid_short_frame);
  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);

  parser.reset();

  payloads = parser.feed_bytes(make_long24_header(1));
  assert(payloads.empty());

  payloads = parser.feed_bytes(valid_short_frame);
  assert(payloads.size() == 1);
  assert(payloads.front() == expected_payload);
}

void test_packet_parser_rejects_invalid_frames() {
  VescPacketParser parser;

  const auto valid = VescProtocol::build_fw_version_request();

  std::vector<std::uint8_t> zero_length_then_valid{0x02, 0x00};
  zero_length_then_valid.insert(zero_length_then_valid.end(), valid.begin(), valid.end());
  auto payloads = parser.feed_bytes(zero_length_then_valid);
  assert(payloads.size() == 1);
  const std::vector<std::uint8_t> expected_fw_payload{0x00};
  assert(payloads.front() == expected_fw_payload);

  parser.reset();

  auto truncated = valid;
  truncated.pop_back();
  assert(parser.feed_bytes(truncated).empty());
}

} // namespace

int main() {
  test_build_requests_exact_bytes();
  test_build_management_requests_exact_bytes();
  test_management_protocol_validation();
  test_build_control_commands_exact_bytes();
  test_control_builders_reject_invalid_floats();
  test_parse_fw_version();
  test_parse_get_values();
  test_parse_get_imu_data_full_mask();
  test_parse_get_imu_data_sparse_mask_and_rejections();
  test_packet_parser_round_trip_and_resync();
  test_packet_parser_short_frame_incremental_delivery();
  test_packet_parser_waits_for_outer_frame_with_nested_frame_bytes();
  test_packet_parser_emits_multiple_frames_from_one_burst();
  test_packet_parser_resyncs_after_garbage_prefix();
  test_packet_parser_frame_boundaries();
  test_packet_parser_resyncs_after_bad_crc();
  test_packet_parser_resyncs_after_bad_stop_byte();
  test_packet_parser_resyncs_after_corrupt_length();
  test_packet_parser_reset_clears_partial_frame_state();
  test_packet_parser_rejects_invalid_medium_frame_lengths();
  test_packet_parser_resyncs_after_unsupported_24bit_sequences();
  test_packet_parser_rejects_invalid_frames();
  return 0;
}
