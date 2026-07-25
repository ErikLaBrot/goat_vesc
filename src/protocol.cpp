#include "goat_motor_controller/controller_protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <type_traits>

namespace goat_motor_controller {

// ── Internal helpers ──────────────────────────────────────────────────────────

namespace {

std::uint16_t read_u16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

std::int16_t read_i16(const std::uint8_t* p) {
  return static_cast<std::int16_t>(read_u16(p));
}

std::uint32_t read_u32(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
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

template <typename T> void append_integral_be(ControllerProtocol::Payload& payload, T value) {
  static_assert(std::is_integral_v<T>, "append_integral_be requires an integral type");

  using U = std::make_unsigned_t<T>;
  const U raw = static_cast<U>(value);
  for (int i = sizeof(U) - 1; i >= 0; --i) {
    payload.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFFU));
  }
}

template <typename T> std::optional<T> scaled_integer(float value, double scale) {
  const double scaled = static_cast<double>(value) * scale;
  if (!std::isfinite(scaled) || scaled < static_cast<double>(std::numeric_limits<T>::lowest()) ||
      scaled > static_cast<double>(std::numeric_limits<T>::max())) {
    return std::nullopt;
  }
  return static_cast<T>(scaled);
}

template <typename T> std::optional<T> scaled_integer_rounded(float value, double scale) {
  const double scaled = std::round(static_cast<double>(value) * scale);
  if (!std::isfinite(scaled) || scaled < static_cast<double>(std::numeric_limits<T>::lowest()) ||
      scaled > static_cast<double>(std::numeric_limits<T>::max())) {
    return std::nullopt;
  }
  return static_cast<T>(scaled);
}

template <typename Image>
std::optional<Image> parse_config_image(const ControllerProtocol::Payload& payload,
                                        CommandId expected_id) {
  constexpr std::size_t kSignatureBytes = 4;
  if (payload.size() < 1 + kSignatureBytes || payload.size() > kMaxPayloadBytes ||
      payload[0] != static_cast<std::uint8_t>(expected_id)) {
    return std::nullopt;
  }
  return Image{{payload.begin() + 1, payload.end()}};
}

} // namespace

// ── Framing ───────────────────────────────────────────────────────────────────

std::uint16_t ControllerProtocol::crc16ccitt(const Payload& data) noexcept {
  std::uint16_t crc = 0;
  for (const auto byte : data) {
    crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint32_t>(byte) << 8U));
    for (int i = 0; i < 8; ++i) {
      const auto shifted = static_cast<std::uint32_t>(crc) << 1U;
      crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>(shifted ^ 0x1021U)
                                  : static_cast<std::uint16_t>(shifted);
    }
  }
  return crc;
}

ControllerProtocol::Payload ControllerProtocol::frame(const Payload& payload) {
  const std::size_t len = payload.size();
  if (len == 0 || len > kMaxPayloadBytes) {
    return {};
  }

  Payload out;
  out.reserve(len + 6);

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

ControllerProtocol::Payload ControllerProtocol::build_fw_version_request() {
  return frame({static_cast<std::uint8_t>(CommandId::FwVersion)});
}

ControllerProtocol::Payload ControllerProtocol::build_get_values_request() {
  return frame({static_cast<std::uint8_t>(CommandId::GetValues)});
}

ControllerProtocol::Payload ControllerProtocol::build_get_imu_data_request() {
  Payload payload{static_cast<std::uint8_t>(CommandId::GetImuData)};
  append_integral_be(payload, std::uint16_t{0xFFFFU});
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_get_motor_config_request() {
  return frame({static_cast<std::uint8_t>(CommandId::GetMotorConfig)});
}

ControllerProtocol::Payload
ControllerProtocol::build_set_motor_config_request(const MotorConfigImage& image) {
  constexpr std::size_t kSignatureBytes = 4;
  if (image.bytes.size() < kSignatureBytes || image.bytes.size() >= kMaxPayloadBytes) {
    return {};
  }

  Payload payload{static_cast<std::uint8_t>(CommandId::SetMotorConfig)};
  payload.insert(payload.end(), image.bytes.begin(), image.bytes.end());
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_get_app_config_request() {
  return frame({static_cast<std::uint8_t>(CommandId::GetAppConfig)});
}

ControllerProtocol::Payload ControllerProtocol::build_set_app_config_request(const AppConfigImage& image,
                                                                 AppConfigStorage storage) {
  constexpr std::size_t kSignatureBytes = 4;
  if (image.bytes.size() < kSignatureBytes || image.bytes.size() >= kMaxPayloadBytes) {
    return {};
  }

  CommandId id;
  switch (storage) {
  case AppConfigStorage::Volatile:
    id = CommandId::SetAppConfigNoStore;
    break;
  case AppConfigStorage::Persistent:
    id = CommandId::SetAppConfig;
    break;
  default:
    return {};
  }
  Payload payload{static_cast<std::uint8_t>(id)};
  payload.insert(payload.end(), image.bytes.begin(), image.bytes.end());
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_lisp_read_request(std::uint32_t length,
                                                            std::uint32_t offset) {
  constexpr std::uint32_t kMaxReadChunk = kMaxPayloadBytes - 10U;
  if (length == 0 || length > kMaxReadChunk ||
      static_cast<std::uint64_t>(length) + offset >
          static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    return {};
  }

  Payload payload{static_cast<std::uint8_t>(CommandId::LispReadCode)};
  append_integral_be(payload, static_cast<std::int32_t>(length));
  append_integral_be(payload, static_cast<std::int32_t>(offset));
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_lisp_erase_request(std::uint32_t size) {
  if (size == 0 || size > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    return {};
  }

  Payload payload{static_cast<std::uint8_t>(CommandId::LispEraseCode)};
  append_integral_be(payload, static_cast<std::int32_t>(size));
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_lisp_write_request(const Payload& chunk,
                                                             std::uint32_t offset) {
  constexpr std::size_t kWriteHeaderBytes = 5;
  if (chunk.empty() || chunk.size() > kMaxPayloadBytes - kWriteHeaderBytes) {
    return {};
  }

  Payload payload{static_cast<std::uint8_t>(CommandId::LispWriteCode)};
  append_integral_be(payload, offset);
  payload.insert(payload.end(), chunk.begin(), chunk.end());
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_lisp_set_running_request(bool running) {
  return frame({
      static_cast<std::uint8_t>(CommandId::LispSetRunning),
      static_cast<std::uint8_t>(running),
  });
}

ControllerProtocol::Payload
ControllerProtocol::build_foc_calibration_request(const FocCalibrationParameters& parameters) {
  constexpr float kMaxPowerLossW = 99999.0f;
  constexpr float kMaxInputCurrentA = 9999.0f;
  constexpr float kMaxErpm = 999999.0f;
  if (!(parameters.max_power_loss_w > 0.0f) ||
      parameters.max_power_loss_w > kMaxPowerLossW ||
      parameters.min_input_current_a < -kMaxInputCurrentA ||
      parameters.min_input_current_a > 0.0f ||
      parameters.max_input_current_a < 0.0f ||
      parameters.max_input_current_a > kMaxInputCurrentA ||
      parameters.openloop_erpm < 0.0f || parameters.openloop_erpm > kMaxErpm ||
      parameters.sensorless_erpm < 0.0f || parameters.sensorless_erpm > kMaxErpm) {
    return {};
  }

  const auto max_loss =
      scaled_integer_rounded<std::int32_t>(parameters.max_power_loss_w, 1000.0);
  const auto min_current =
      scaled_integer_rounded<std::int32_t>(parameters.min_input_current_a, 1000.0);
  const auto max_current =
      scaled_integer_rounded<std::int32_t>(parameters.max_input_current_a, 1000.0);
  const auto openloop =
      scaled_integer_rounded<std::int32_t>(parameters.openloop_erpm, 1000.0);
  const auto sensorless =
      scaled_integer_rounded<std::int32_t>(parameters.sensorless_erpm, 1000.0);
  if (!max_loss || !min_current || !max_current || !openloop || !sensorless) {
    return {};
  }

  Payload payload{static_cast<std::uint8_t>(CommandId::DetectApplyAllFoc), 0};
  append_integral_be(payload, *max_loss);
  append_integral_be(payload, *min_current);
  append_integral_be(payload, *max_current);
  append_integral_be(payload, *openloop);
  append_integral_be(payload, *sensorless);
  return frame(payload);
}

ControllerProtocol::Payload
ControllerProtocol::build_app_disable_output_command(std::chrono::milliseconds duration) {
  if (duration < std::chrono::milliseconds::zero() ||
      duration.count() > std::numeric_limits<std::int32_t>::max()) {
    return {};
  }

  Payload payload{static_cast<std::uint8_t>(CommandId::AppDisableOutput), 0};
  append_integral_be(payload, static_cast<std::int32_t>(duration.count()));
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::pack_lisp_code(const LispCodeImage& image) {
  if (image.bytes.empty() || image.bytes.size() > kMaxLispCodeBytes) {
    return {};
  }

  Payload code{0, 0}; // Current firmware code-image flags.
  code.insert(code.end(), image.bytes.begin(), image.bytes.end());

  Payload packed;
  packed.reserve(code.size() + 6);
  append_integral_be(packed, static_cast<std::uint32_t>(image.bytes.size()));
  append_integral_be(packed, crc16ccitt(code));
  packed.insert(packed.end(), code.begin(), code.end());
  return packed;
}

ControllerProtocol::Payload ControllerProtocol::build_set_rpm_command(std::int32_t rpm) {
  Payload payload{static_cast<std::uint8_t>(CommandId::SetRpm)};
  append_integral_be(payload, rpm);
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_set_duty_command(float duty) {
  const auto scaled = scaled_integer<std::int32_t>(duty, 100000.0);
  if (!scaled || duty < -1.0f || duty > 1.0f) {
    return {};
  }
  Payload payload{static_cast<std::uint8_t>(CommandId::SetDuty)};
  append_integral_be(payload, *scaled);
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_set_current_command(float amps) {
  const auto scaled = scaled_integer<std::int32_t>(amps, 1000.0);
  if (!scaled) {
    return {};
  }
  Payload payload{static_cast<std::uint8_t>(CommandId::SetCurrent)};
  append_integral_be(payload, *scaled);
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_set_current_brake_command(float amps) {
  const auto scaled = scaled_integer<std::int32_t>(amps, 1000.0);
  if (!scaled) {
    return {};
  }
  Payload payload{static_cast<std::uint8_t>(CommandId::SetCurrentBrake)};
  append_integral_be(payload, *scaled);
  return frame(payload);
}

ControllerProtocol::Payload ControllerProtocol::build_set_servo_pos_command(float position) {
  const auto scaled = scaled_integer<std::int16_t>(position, 1000.0);
  if (!scaled || position < 0.0f || position > 1.0f) {
    return {};
  }
  Payload payload{static_cast<std::uint8_t>(CommandId::SetServoPos)};
  append_integral_be(payload, *scaled);
  return frame(payload);
}

// ── Response parsers ──────────────────────────────────────────────────────────

std::optional<FwVersion> ControllerProtocol::parse_fw_version(const Payload& payload) {
  // [0] comm_id  [1] major  [2] minor  [3..] hw name, uuid, ...
  if (payload.size() < 3) {
    return std::nullopt;
  }
  if (payload[0] != static_cast<std::uint8_t>(CommandId::FwVersion)) {
    return std::nullopt;
  }
  return FwVersion{payload[1], payload[2]};
}

std::optional<MotorState> ControllerProtocol::parse_get_values(const Payload& payload) {
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
  // Firmware 7.00 optionally continues with PID position (4), controller ID
  // (1), three MOSFET temperatures (6), Vd (4), Vq (4), and status (1).
  // Status bit 0 is the command timeout and bit 1 is the kill-switch latch.
  constexpr std::size_t kMinLen = 1 + 53; // comm_id + 53 data bytes
  constexpr std::size_t kStatusOffset = 72;
  constexpr std::size_t kStatusLen = 1 + kStatusOffset + 1;
  if (payload.size() < kMinLen) {
    return std::nullopt;
  }
  if (payload[0] != static_cast<std::uint8_t>(CommandId::GetValues)) {
    return std::nullopt;
  }

  const auto* p = payload.data() + 1;

  MotorState s;
  s.temp_fet = static_cast<float>(read_i16(p + 0)) / 10.0f;
  s.temp_motor = static_cast<float>(read_i16(p + 2)) / 10.0f;
  s.current_motor = static_cast<float>(read_i32(p + 4)) / 100.0f;
  s.current_in = static_cast<float>(read_i32(p + 8)) / 100.0f;
  // skip id (+12), iq (+16)
  s.duty_cycle = static_cast<float>(read_i16(p + 20)) / 1000.0f;
  s.rpm = static_cast<float>(read_i32(p + 22));
  s.vin = static_cast<float>(read_i16(p + 26)) / 10.0f;
  // skip amp_hours (+28), amp_hours_charged (+32), watt_hours (+36), watt_hours_charged (+40)
  s.tachometer = read_i32(p + 44);
  s.tachometer_abs = read_i32(p + 48);
  s.fault_code = p[52];
  if (payload.size() >= kStatusLen) {
    s.has_timeout = (p[kStatusOffset] & 0x01U) != 0U;
    s.kill_switch_active = (p[kStatusOffset] & 0x02U) != 0U;
  }

  return s;
}

std::optional<ImuData> ControllerProtocol::parse_get_imu_data(const Payload& payload) {
  // Layout:
  //   [0]      comm_id  (GetImuData = 65)
  //   [1..2]   mask     uint16 — which fields follow
  //   [3..]    one 4-byte big-endian IEEE 754 float per set bit, in bit order:
  //              bit 0  Roll     bit 4  AccY    bit  8  GyroZ   bit 12  QuatW
  //              bit 1  Pitch    bit 5  AccZ    bit  9  MagX    bit 13  QuatX
  //              bit 2  Yaw      bit 6  GyroX   bit 10  MagY    bit 14  QuatY
  //              bit 3  AccX     bit 7  GyroY   bit 11  MagZ    bit 15  QuatZ
  if (payload.size() < 3)
    return std::nullopt;
  if (payload[0] != static_cast<std::uint8_t>(CommandId::GetImuData))
    return std::nullopt;

  const std::uint16_t mask = read_u16(payload.data() + 1);
  const auto field_count = static_cast<std::size_t>(__builtin_popcount(mask));

  if (payload.size() < 3 + field_count * 4)
    return std::nullopt;

  const auto* p = payload.data() + 3;
  ImuData d;

  auto next = [&]() -> float {
    const float v = read_f32(p);
    p += 4;
    return v;
  };

  std::uint32_t field_bit = 1U;
  const auto next_if_present = [&](float& field) {
    if ((mask & field_bit) != 0U)
      field = next();
    field_bit <<= 1U;
  };

  next_if_present(d.roll);
  next_if_present(d.pitch);
  next_if_present(d.yaw);
  next_if_present(d.acc_x);
  next_if_present(d.acc_y);
  next_if_present(d.acc_z);
  next_if_present(d.gyro_x);
  next_if_present(d.gyro_y);
  next_if_present(d.gyro_z);
  next_if_present(d.mag_x);
  next_if_present(d.mag_y);
  next_if_present(d.mag_z);
  next_if_present(d.quat_w);
  next_if_present(d.quat_x);
  next_if_present(d.quat_y);
  next_if_present(d.quat_z);

  return d;
}

std::optional<MotorConfigImage> ControllerProtocol::parse_motor_config(const Payload& payload) {
  return parse_config_image<MotorConfigImage>(payload, CommandId::GetMotorConfig);
}

std::optional<AppConfigImage> ControllerProtocol::parse_app_config(const Payload& payload) {
  return parse_config_image<AppConfigImage>(payload, CommandId::GetAppConfig);
}

std::optional<ControllerProtocol::Payload>
ControllerProtocol::parse_lisp_read_reply(const Payload& payload, std::uint32_t& total_size,
                                    std::uint32_t& offset) {
  constexpr std::size_t kHeaderBytes = 9;
  if (payload.size() < kHeaderBytes ||
      payload[0] != static_cast<std::uint8_t>(CommandId::LispReadCode)) {
    return std::nullopt;
  }

  const auto total = read_i32(payload.data() + 1);
  const auto chunk_offset = read_i32(payload.data() + 5);
  const auto chunk_size = payload.size() - kHeaderBytes;
  if (total < 0 || chunk_offset < 0 || static_cast<std::size_t>(total) > kMaxLispCodeBytes ||
      static_cast<std::uint64_t>(chunk_offset) + chunk_size >
          static_cast<std::uint64_t>(total) ||
      (total == 0 && (chunk_offset != 0 || chunk_size != 0))) {
    return std::nullopt;
  }

  total_size = static_cast<std::uint32_t>(total);
  offset = static_cast<std::uint32_t>(chunk_offset);
  return Payload(payload.begin() + static_cast<std::ptrdiff_t>(kHeaderBytes), payload.end());
}

bool ControllerProtocol::parse_config_ack(const Payload& payload, CommandId expected_id) {
  return payload.size() == 1 && payload[0] == static_cast<std::uint8_t>(expected_id);
}

bool ControllerProtocol::parse_lisp_write_ack(const Payload& payload, std::uint32_t expected_offset) {
  return payload.size() == 6 &&
         payload[0] == static_cast<std::uint8_t>(CommandId::LispWriteCode) &&
         payload[1] == 1 && read_u32(payload.data() + 2) == expected_offset;
}

bool ControllerProtocol::parse_bool_ack(const Payload& payload, CommandId expected_id) {
  return payload.size() == 2 && payload[0] == static_cast<std::uint8_t>(expected_id) &&
         payload[1] == 1;
}

std::optional<std::int16_t>
ControllerProtocol::parse_foc_calibration_reply(const Payload& payload) {
  if (payload.size() != 3 ||
      payload[0] != static_cast<std::uint8_t>(CommandId::DetectApplyAllFoc)) {
    return std::nullopt;
  }
  return read_i16(payload.data() + 1);
}

} // namespace goat_motor_controller
