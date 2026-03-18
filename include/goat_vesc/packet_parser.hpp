#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace goat_vesc {

class VescPacketParser {
public:
  using Payload = std::vector<std::uint8_t>;

  VescPacketParser() = default;

  void reset();

  std::optional<Payload> feed_byte(std::uint8_t byte);
  std::vector<Payload> feed_bytes(const std::vector<std::uint8_t>& bytes);

private:
  std::vector<std::uint8_t> buffer_;

  enum class DecodeResult { Success, NeedMoreData, Invalid };

  DecodeResult try_decode_packet_(Payload& payload_out);
  static std::uint16_t crc16ccitt_(const std::vector<std::uint8_t>& data) noexcept;
};

} // namespace goat_vesc
