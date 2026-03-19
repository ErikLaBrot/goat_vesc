/**
 * @file packet_parser.hpp
 * @brief Incremental parser for framed VESC serial packets.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace goat_vesc {

/**
 * @brief Incrementally decodes framed VESC packets from a byte stream.
 *
 * The parser owns an internal buffer and can be fed one byte at a time or in
 * batches. It validates framing and CRC before returning a payload.
 */
class VescPacketParser {
public:
  /** @brief Framing-free VESC packet payload. */
  using Payload = std::vector<std::uint8_t>;

  /** @brief Creates an empty parser with no buffered bytes. */
  VescPacketParser() = default;

  /** @brief Clears buffered state and restarts parsing from an empty stream. */
  void reset();

  /**
   * @brief Feeds one byte of transport data into the parser.
   * @param byte Raw byte read from the transport.
   * @return A decoded payload when a full valid packet is available.
   */
  std::optional<Payload> feed_byte(std::uint8_t byte);
  /**
   * @brief Feeds a batch of transport bytes into the parser.
   * @param bytes Raw bytes read from the transport.
   * @return Every decoded payload completed while consuming the batch.
   */
  std::vector<Payload> feed_bytes(const std::vector<std::uint8_t>& bytes);

private:
  std::vector<std::uint8_t> buffer_;

  enum class DecodeResult { Success, NeedMoreData, Invalid };

  DecodeResult try_decode_packet_(Payload& payload_out, std::size_t& discard_bytes_out);
  static std::uint16_t crc16ccitt_(const std::vector<std::uint8_t>& data) noexcept;
};

} // namespace goat_vesc
