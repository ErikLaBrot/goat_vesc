/*
 * Probe example for bring-up and smoke testing.
 *
 * Demonstrates:
 * - connecting to a VESC
 * - querying firmware version
 * - waiting for IMU and motor-state samples
 * - printing decoded telemetry fields
 */

#include "goat_vesc/vesc_client.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

using namespace goat_vesc;
using namespace std::chrono_literals;

namespace {

void print_usage(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [device_path] [baud]\n"
            << "  device_path: serial device such as /dev/ttyACM0\n"
            << "  baud: supported baud rate, default 115200\n";
}

template <typename Getter> bool wait_for_sample(Getter getter, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (getter().has_value()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return getter().has_value();
}

void print_fw(const std::optional<FwVersion>& fw) {
  if (!fw) {
    std::cout << "Firmware: timeout or no response\n";
    return;
  }

  std::cout << "Firmware: " << static_cast<int>(fw->major) << "." << static_cast<int>(fw->minor)
            << '\n';
}

void print_imu(const std::optional<VescIMUData>& imu) {
  if (!imu) {
    std::cout << "IMU: no sample received\n";
    return;
  }

  std::cout << "IMU sample:\n"
            << "  stamp_ns: " << imu->stamp_ns << '\n'
            << "  roll/pitch/yaw: " << imu->roll << ", " << imu->pitch << ", " << imu->yaw << '\n'
            << "  acc xyz: " << imu->acc_x << ", " << imu->acc_y << ", " << imu->acc_z << '\n'
            << "  gyro xyz: " << imu->gyro_x << ", " << imu->gyro_y << ", " << imu->gyro_z << '\n';
}

void print_motor_state(const std::optional<VescMotorState>& state) {
  if (!state) {
    std::cout << "Motor state: no sample received\n";
    return;
  }

  std::cout << "Motor state sample:\n"
            << "  stamp_ns: " << state->stamp_ns << '\n'
            << "  rpm: " << state->rpm << '\n'
            << "  duty: " << state->duty_cycle << '\n'
            << "  current_motor/current_in: " << state->current_motor << ", " << state->current_in
            << '\n'
            << "  vin: " << state->vin << '\n'
            << "  temp_motor/temp_fet: " << state->temp_motor << ", " << state->temp_fet << '\n'
            << "  fault_code: " << static_cast<int>(state->fault_code) << '\n';
}

} // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 1) {
      const std::string arg1 = argv[1];
      if (arg1 == "-h" || arg1 == "--help") {
        print_usage(argv[0]);
        return 0;
      }
    }

    VescConfig config;
    if (argc > 1) {
      config.device_path = argv[1];
    }
    if (argc > 2) {
      config.baud = std::stoi(argv[2]);
    }

    config.imu_poll_interval = 20ms;
    config.motor_poll_interval = 50ms;
    config.poll_response_timeout = 50ms;
    config.query_guard_window = 5ms;

    std::cout << "VESC probe starting\n";
    if (!config.device_path.empty()) {
      std::cout << "Device: " << config.device_path << '\n';
    } else {
      std::cout << "Device: auto-detect first /dev/ttyACM*\n";
    }
    std::cout << "Baud: " << config.baud << '\n';

    VescClient client(config);
    if (!client.connect()) {
      std::cerr << "Connect failed\n";
      return 2;
    }

    std::cout << "Connected\n";

    const auto fw = client.request_fw_version(300ms);
    print_fw(fw);

    const bool got_imu = wait_for_sample([&client] { return client.latest_imu(); }, 750ms);
    const bool got_motor =
        wait_for_sample([&client] { return client.latest_motor_state(); }, 750ms);

    print_imu(client.latest_imu());
    print_motor_state(client.latest_motor_state());

    client.disconnect();

    if (!fw || !got_imu || !got_motor) {
      std::cerr << "Probe incomplete\n";
      return 3;
    }

    std::cout << "Probe OK\n";
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Probe error: " << ex.what() << '\n';
    return 4;
  }
}
