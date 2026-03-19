/*
 * Duty sweep example.
 *
 * Demonstrates:
 * - issuing a sequence of `set_duty(...)` commands
 * - checking connectivity with a firmware query
 * - reading motor-state telemetry before and after the sweep
 *
 * This touches real hardware and is intended as a manual example/smoke test.
 */

#include "goat_vesc/vesc_client.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace goat_vesc;
using namespace std::chrono_literals;

namespace {

void print_usage(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [device_path] [baud]\n"
            << "  device_path: serial device such as /dev/ttyACM0\n"
            << "  baud: supported baud rate, default 115200\n";
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

    config.imu_poll_interval = 0ms;
    config.motor_poll_interval = 50ms;
    config.poll_response_timeout = 100ms;

    VescClient client(config);
    if (!client.connect()) {
      std::cerr << "Failed to connect to VESC\n";
      return 2;
    }

    std::cout << "Connected";
    if (!config.device_path.empty()) {
      std::cout << " to " << config.device_path;
    }
    std::cout << " at " << config.baud << " baud\n";

    if (auto fw = client.request_fw_version(300ms)) {
      std::cout << "Firmware: " << static_cast<int>(fw->major) << "." << static_cast<int>(fw->minor)
                << '\n';
    } else {
      std::cout << "Firmware query timed out\n";
    }

    if (auto state = client.latest_motor_state()) {
      std::cout << "Initial motor state rpm=" << state->rpm << " duty=" << state->duty_cycle
                << " vin=" << state->vin << '\n';
    } else {
      std::cout << "Initial motor state not available yet\n";
    }

    const std::vector<float> sweep{
        0.00f, 0.05f, 0.10f, 0.15f, 0.10f, 0.05f, 0.00f, -0.05f, -0.10f, -0.05f, 0.00f,
    };

    std::cout << "Starting duty sweep\n";
    for (const float duty : sweep) {
      std::cout << "  set_duty(" << duty << ")\n";
      if (!client.set_duty(duty)) {
        std::cerr << "Failed to send duty command\n";
        (void)client.set_duty(0.0f);
        client.disconnect();
        return 3;
      }
      std::this_thread::sleep_for(750ms);
    }

    std::cout << "Commanding final zero duty\n";
    if (!client.set_duty(0.0f)) {
      std::cerr << "Failed to send final zero duty command\n";
    }
    std::this_thread::sleep_for(300ms);

    if (auto state = client.latest_motor_state()) {
      std::cout << "Final motor state rpm=" << state->rpm << " duty=" << state->duty_cycle
                << " vin=" << state->vin << '\n';
    }

    client.disconnect();
    std::cout << "Duty sweep complete\n";
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Duty sweep error: " << ex.what() << '\n';
    return 4;
  }
}
