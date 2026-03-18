#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace test_support {

inline std::uint16_t crc16ccitt(const std::vector<std::uint8_t>& data) {
  std::uint16_t crc = 0;
  for (const auto byte : data) {
    crc ^= static_cast<std::uint16_t>(byte) << 8;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline void append_u8(std::vector<std::uint8_t>& payload, std::uint8_t value) {
  payload.push_back(value);
}

inline void append_u16(std::vector<std::uint8_t>& payload, std::uint16_t value) {
  payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
  payload.push_back(static_cast<std::uint8_t>(value & 0xFF));
}

inline void append_i16(std::vector<std::uint8_t>& payload, std::int16_t value) {
  append_u16(payload, static_cast<std::uint16_t>(value));
}

inline void append_i32(std::vector<std::uint8_t>& payload, std::int32_t value) {
  const auto raw = static_cast<std::uint32_t>(value);
  payload.push_back(static_cast<std::uint8_t>((raw >> 24) & 0xFF));
  payload.push_back(static_cast<std::uint8_t>((raw >> 16) & 0xFF));
  payload.push_back(static_cast<std::uint8_t>((raw >> 8) & 0xFF));
  payload.push_back(static_cast<std::uint8_t>(raw & 0xFF));
}

inline void append_f32(std::vector<std::uint8_t>& payload, float value) {
  std::uint32_t raw = 0;
  static_assert(sizeof(raw) == sizeof(value));
  std::memcpy(&raw, &value, sizeof(raw));
  append_i32(payload, static_cast<std::int32_t>(raw));
}

inline std::vector<std::uint8_t> frame_payload(const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> framed;
  if (payload.size() <= 255) {
    framed.push_back(0x02);
    framed.push_back(static_cast<std::uint8_t>(payload.size()));
  } else {
    framed.push_back(0x03);
    framed.push_back(static_cast<std::uint8_t>((payload.size() >> 8) & 0xFF));
    framed.push_back(static_cast<std::uint8_t>(payload.size() & 0xFF));
  }

  framed.insert(framed.end(), payload.begin(), payload.end());

  const auto crc = crc16ccitt(payload);
  framed.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
  framed.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  framed.push_back(0x03);

  return framed;
}

} // namespace test_support
