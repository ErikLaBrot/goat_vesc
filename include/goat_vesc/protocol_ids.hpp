/*
* Protocol info and cmd values for vesc. Anything like bitmasks or packet id goes here.
*/

//To do: extend packetcommid with full range of values

#pragma once

#include <cstdint>

namespace goat_vesc{

    // ── Firmware-defined packet size limits ───────────────────────────────────
    // These are spec constants, not implementation choices. Update here if a
    // future firmware version raises the payload ceiling.

    // Maximum payload length the VESC firmware will accept or produce.
    constexpr std::size_t kMaxPayloadBytes = 512;

    // Maximum size of a fully framed packet on the wire:
    //   1 start + 2 length (16-bit, since 512 > 255) + payload + 2 CRC + 1 stop
    constexpr std::size_t kMaxFramedPacketBytes = kMaxPayloadBytes + 6;

    enum class VescPacketLength : std::uint8_t {
        Short = 0x02,
        Medium = 0x03,
        Long = 0x04,
    };

    enum class VescPacketCommID : std::uint8_t {
        FwVersion = 0,
        JumpToBootloader = 1,
        EraseNewApp = 2,
        WriteNewAppData = 3,
        GetValues = 4,
        SetDuty = 5,
        SetCurrent = 6,
        SetCurrentBrake = 7,
        SetRpm = 8,
        SetPos = 9,
        SetHandbrake = 10,
        SetDetect = 11,
        SetServoPos = 12,
        GetImuData = 65,
    };

    enum class VescValuesMask : std::uint32_t {
        FetTemp = (1u << 0),
        MotorTemp = (1u << 1),
        CurrentMotor = (1u << 2),
        CurrentIn = (1u << 3),
        CurrentId = (1u << 4),
        CurrentIq = (1u << 5),
        DutyCycle = (1u << 6),
        Rpm = (1u << 7),
        Vin = (1u << 8),
        AmpHours = (1u << 9),
        AmpHoursCharged = (1u << 10),
        WattHours = (1u << 11),
        WattHoursCharged = (1u << 12),
        Tachometer = (1u << 13),
        TachometerAbs = (1u << 14),
        FaultCode = (1u << 15),
        PidPosNow = (1u << 16),
        ControllerId = (1u << 17),
        MosTemps = (1u << 18),
        Vd = (1u << 19),
        Vq = (1u << 20),
        StatusFlags = (1u << 21)
    };

    constexpr std::uint32_t operator|(VescValuesMask lhs, VescValuesMask rhs) {
        return static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs);
    }

    constexpr std::uint32_t DefaultVescValuesMask =
        static_cast<std::uint32_t>(VescValuesMask::FetTemp) |
        static_cast<std::uint32_t>(VescValuesMask::MotorTemp) |
        static_cast<std::uint32_t>(VescValuesMask::CurrentMotor) |
        static_cast<std::uint32_t>(VescValuesMask::CurrentIn) |
        static_cast<std::uint32_t>(VescValuesMask::CurrentId) |
        static_cast<std::uint32_t>(VescValuesMask::CurrentIq) |
        static_cast<std::uint32_t>(VescValuesMask::DutyCycle) |
        static_cast<std::uint32_t>(VescValuesMask::Rpm) |
        static_cast<std::uint32_t>(VescValuesMask::Vin) |
        static_cast<std::uint32_t>(VescValuesMask::AmpHours) |
        static_cast<std::uint32_t>(VescValuesMask::AmpHoursCharged) |
        static_cast<std::uint32_t>(VescValuesMask::WattHours) |
        static_cast<std::uint32_t>(VescValuesMask::WattHoursCharged) |
        static_cast<std::uint32_t>(VescValuesMask::Tachometer) |
        static_cast<std::uint32_t>(VescValuesMask::TachometerAbs) |
        static_cast<std::uint32_t>(VescValuesMask::FaultCode) |
        static_cast<std::uint32_t>(VescValuesMask::PidPosNow) |
        static_cast<std::uint32_t>(VescValuesMask::ControllerId) |
        static_cast<std::uint32_t>(VescValuesMask::MosTemps) |
        static_cast<std::uint32_t>(VescValuesMask::Vd) |
        static_cast<std::uint32_t>(VescValuesMask::Vq) |
        static_cast<std::uint32_t>(VescValuesMask::StatusFlags);

    enum class VescImuMask : std::uint16_t {
        Roll = (1u << 0),
        Pitch = (1u << 1),
        Yaw = (1u << 2),
        AccX = (1u << 3),
        AccY = (1u << 4),
        AccZ = (1u << 5),
        GyroX = (1u << 6),
        GyroY = (1u << 7),
        GyroZ = (1u << 8),
        MagX = (1u << 9),
        MagY = (1u << 10),
        MagZ = (1u << 11),
        QuatW = (1u << 12),
        QuatX = (1u << 13),
        QuatY = (1u << 14),
        QuatZ = (1u << 15)
    };

    constexpr std::uint16_t operator|(VescImuMask lhs, VescImuMask rhs) {
        return static_cast<std::uint16_t>(lhs) | static_cast<std::uint16_t>(rhs);
    }

    constexpr std::uint16_t DefaultVescImuMask =
        static_cast<std::uint16_t>(VescImuMask::Roll) |
        static_cast<std::uint16_t>(VescImuMask::Pitch) |
        static_cast<std::uint16_t>(VescImuMask::Yaw) |
        static_cast<std::uint16_t>(VescImuMask::AccX) |
        static_cast<std::uint16_t>(VescImuMask::AccY) |
        static_cast<std::uint16_t>(VescImuMask::AccZ) |
        static_cast<std::uint16_t>(VescImuMask::GyroX) |
        static_cast<std::uint16_t>(VescImuMask::GyroY) |
        static_cast<std::uint16_t>(VescImuMask::GyroZ) |
        static_cast<std::uint16_t>(VescImuMask::MagX) |
        static_cast<std::uint16_t>(VescImuMask::MagY) |
        static_cast<std::uint16_t>(VescImuMask::MagZ) |
        static_cast<std::uint16_t>(VescImuMask::QuatW) |
        static_cast<std::uint16_t>(VescImuMask::QuatX) |
        static_cast<std::uint16_t>(VescImuMask::QuatY) |
        static_cast<std::uint16_t>(VescImuMask::QuatZ);


} // namespace goat_vesc 
