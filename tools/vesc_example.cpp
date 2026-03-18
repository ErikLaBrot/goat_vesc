#include "goat_vesc/vesc_client.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace goat_vesc;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  VescConfig config;
  if (argc > 1) {
    config.device_path = argv[1];
  }
  config.imu_poll_interval = 10ms;
  config.motor_poll_interval = 50ms;

  VescClient client(config);
  if (!client.connect()) {
    std::cerr << "Failed to connect to VESC\n";
    return 1;
  }

  auto imu_sub = client.subscribe_imu([](const VescIMUData& imu) {
    std::cout << "IMU stamp_ns=" << imu.stamp_ns << " gyro_z=" << imu.gyro_z
              << " acc_x=" << imu.acc_x << '\n';
  });

  auto motor_sub = client.subscribe_motor_state([](const VescMotorState& state) {
    std::cout << "Motor stamp_ns=" << state.stamp_ns << " rpm=" << state.rpm << " vin=" << state.vin
              << '\n';
  });

  if (auto fw = client.request_fw_version(200ms)) {
    std::cout << "Firmware " << static_cast<int>(fw->major) << "." << static_cast<int>(fw->minor)
              << '\n';
  } else {
    std::cout << "Firmware query timed out\n";
  }

  if (argc > 2) {
    const float current = std::strtof(argv[2], nullptr);
    if (!client.set_current(current)) {
      std::cerr << "Failed to send current command\n";
    }
  }

  std::this_thread::sleep_for(2s);

  if (auto latest = client.latest_motor_state()) {
    std::cout << "Latest cached rpm=" << latest->rpm << " duty=" << latest->duty_cycle << '\n';
  }

  client.disconnect();
  return 0;
}
