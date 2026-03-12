#include "goat_vesc/packet_parser.hpp"
#include "goat_vesc/protocol_ids.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace {

constexpr std::uint8_t kStartShort = 2;
constexpr std::uint8_t kStartLong16 = 3;
constexpr std::uint8_t kStartLong24 = 4;
constexpr std::uint8_t kStopByte = 3;

} // namespace

namespace goat_vesc {

void VescPacketParser::reset() {
    buffer_.clear();
}

std::optional<VescPacketParser::Payload> VescPacketParser::feed_byte(std::uint8_t byte) {
    buffer_.push_back(byte);

    Payload payload;
    for (;;) {
        const auto result = try_decode_packet_(payload);

        if (result == DecodeResult::Success) {
            return payload;
        }

        if (result == DecodeResult::NeedMoreData) {
            return std::nullopt;
        }

        // Invalid: drop one byte and try to resync
        if (!buffer_.empty()) {
            buffer_.erase(buffer_.begin());
        } else {
            return std::nullopt;
        }
    }
}

std::vector<VescPacketParser::Payload> VescPacketParser::feed_bytes(
    const std::vector<std::uint8_t>& bytes) {

    std::vector<Payload> out;

    for (const auto byte : bytes) {
        if (auto payload = feed_byte(byte)) {
            out.push_back(std::move(*payload));
        }
    }

    return out;
}

VescPacketParser::DecodeResult
VescPacketParser::try_decode_packet_(Payload& payload_out) {
    if (buffer_.empty()) {
        return DecodeResult::NeedMoreData;
    }

    const std::uint8_t start = buffer_[0];

    std::size_t header_len = 0;
    std::size_t payload_len = 0;

    if (start == kStartShort) {
        header_len = 2;

        if (buffer_.size() < header_len) {
            return DecodeResult::NeedMoreData;
        }

        payload_len = buffer_[1];

        // VESC rejects zero-length packets
        if (payload_len < 1) {
            return DecodeResult::Invalid;
        }

    } else if (start == kStartLong16) {
        header_len = 3;

        if (buffer_.size() < header_len) {
            return DecodeResult::NeedMoreData;
        }

        payload_len =
            (static_cast<std::size_t>(buffer_[1]) << 8) |
            static_cast<std::size_t>(buffer_[2]);

        // Shorter packets should have used the short format
        if (payload_len < 255 || payload_len > kMaxPayloadBytes) {
            return DecodeResult::Invalid;
        }

    } else if (start == kStartLong24) {
        header_len = 4;

        if (buffer_.size() < header_len) {
            return DecodeResult::NeedMoreData;
        }

        payload_len =
            (static_cast<std::size_t>(buffer_[1]) << 16) |
            (static_cast<std::size_t>(buffer_[2]) << 8) |
            static_cast<std::size_t>(buffer_[3]);

        // Shorter packets should have used the 16-bit format
        if (payload_len < 65535 || payload_len > kMaxPayloadBytes) {
            return DecodeResult::Invalid;
        }

    } else {
        return DecodeResult::Invalid;
    }

    const std::size_t total_len = header_len + payload_len + 2 + 1; // payload + crc + stop

    if (buffer_.size() < total_len) {
        return DecodeResult::NeedMoreData;
    }

    const std::size_t payload_start = header_len;
    const std::size_t crc_hi_index = payload_start + payload_len;
    const std::size_t crc_lo_index = payload_start + payload_len + 1;
    const std::size_t stop_index = payload_start + payload_len + 2;

    if (buffer_[stop_index] != kStopByte) {
        return DecodeResult::Invalid;
    }

    Payload payload(
        buffer_.begin() + static_cast<std::ptrdiff_t>(payload_start),
        buffer_.begin() + static_cast<std::ptrdiff_t>(payload_start + payload_len)
    );

    const std::uint16_t crc_rx =
        (static_cast<std::uint16_t>(buffer_[crc_hi_index]) << 8) |
        static_cast<std::uint16_t>(buffer_[crc_lo_index]);

    const std::uint16_t crc_calc = crc16ccitt_(payload);

    if (crc_rx != crc_calc) {
        return DecodeResult::Invalid;
    }

    buffer_.erase(
        buffer_.begin(),
        buffer_.begin() + static_cast<std::ptrdiff_t>(total_len)
    );

    payload_out = std::move(payload);
    return DecodeResult::Success;
}

std::uint16_t VescPacketParser::crc16ccitt_(const std::vector<std::uint8_t>& data) noexcept {
    std::uint16_t crc = 0;

    for (const auto byte : data) {
        crc ^= static_cast<std::uint16_t>(byte) << 8;

        for (int i = 0; i < 8; ++i) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

} // namespace goat_vesc
