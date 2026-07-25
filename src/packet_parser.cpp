#include "goat_motor_controller/packet_parser.hpp"
#include "goat_motor_controller/protocol_ids.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace {

constexpr std::uint8_t kStartShort = 2;
constexpr std::uint8_t kStartLong16 = 3;
constexpr std::uint8_t kStopByte = 3;

} // namespace

namespace goat_motor_controller {

void PacketParser::reset() {
  buffer_.clear();
}

std::optional<PacketParser::Payload> PacketParser::feed_byte(std::uint8_t byte) {
  buffer_.push_back(byte);

  for (;;) {
    while (!buffer_.empty() && buffer_.front() != kStartShort &&
           buffer_.front() != kStartLong16) {
      buffer_.erase(buffer_.begin());
    }
    if (buffer_.empty()) {
      return std::nullopt;
    }

    Payload payload;
    std::size_t packet_size = 0;
    const auto result = try_decode_packet_(payload, packet_size);
    if (result == DecodeResult::Success) {
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
      return payload;
    }
    if (result == DecodeResult::NeedMoreData) {
      return std::nullopt;
    }

    if (result == DecodeResult::InvalidHeader) {
      buffer_.erase(buffer_.begin());
    } else {
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
    }
    if (buffer_.empty()) {
      return std::nullopt;
    }
  }
}

std::vector<PacketParser::Payload>
PacketParser::feed_bytes(const std::vector<std::uint8_t>& bytes) {

  std::vector<Payload> out;

  for (const auto byte : bytes) {
    if (auto payload = feed_byte(byte)) {
      out.push_back(std::move(*payload));
    }
  }

  return out;
}

PacketParser::DecodeResult
PacketParser::try_decode_packet_(Payload& payload_out, std::size_t& packet_size_out) const {
  const std::size_t available = buffer_.size();
  if (available == 0) {
    return DecodeResult::NeedMoreData;
  }

  const std::uint8_t start = buffer_.front();

  std::size_t header_len = 0;
  std::size_t payload_len = 0;

  if (start == kStartShort) {
    header_len = 2;

    if (available < header_len) {
      return DecodeResult::NeedMoreData;
    }

    packet_size_out = header_len;
    payload_len = buffer_[1];

    // The firmware rejects zero-length packets.
    if (payload_len < 1) {
      return DecodeResult::InvalidHeader;
    }

  } else if (start == kStartLong16) {
    header_len = 3;

    if (available < header_len) {
      return DecodeResult::NeedMoreData;
    }

    packet_size_out = header_len;
    payload_len =
        (static_cast<std::size_t>(buffer_[1]) << 8) | static_cast<std::size_t>(buffer_[2]);

    // Shorter packets should have used the short format
    if (payload_len <= 255 || payload_len > kMaxPayloadBytes) {
      return DecodeResult::InvalidHeader;
    }

  } else {
    // With the current 512-byte payload ceiling, 0x04 is not a supported frame
    // start byte. Drop it like any other unsupported prefix so the parser can
    // resync on the next byte.
    packet_size_out = 1;
    return DecodeResult::InvalidHeader;
  }

  const std::size_t total_len = header_len + payload_len + 2 + 1; // payload + crc + stop
  packet_size_out = total_len;

  if (available < total_len) {
    return DecodeResult::NeedMoreData;
  }

  const std::size_t payload_start = header_len;
  const std::size_t crc_hi_index = payload_start + payload_len;
  const std::size_t crc_lo_index = payload_start + payload_len + 1;
  const std::size_t stop_index = payload_start + payload_len + 2;

  if (buffer_[stop_index] != kStopByte) {
    return DecodeResult::InvalidPacket;
  }

  Payload payload(buffer_.begin() + static_cast<std::ptrdiff_t>(payload_start),
                  buffer_.begin() + static_cast<std::ptrdiff_t>(payload_start + payload_len));

  const auto crc_rx =
      static_cast<std::uint16_t>((static_cast<std::uint32_t>(buffer_[crc_hi_index]) << 8U) |
                                 static_cast<std::uint32_t>(buffer_[crc_lo_index]));

  const std::uint16_t crc_calc = crc16ccitt_(payload);

  if (crc_rx != crc_calc) {
    return DecodeResult::InvalidPacket;
  }

  payload_out = std::move(payload);
  return DecodeResult::Success;
}

std::uint16_t PacketParser::crc16ccitt_(const std::vector<std::uint8_t>& data) noexcept {
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

} // namespace goat_motor_controller
