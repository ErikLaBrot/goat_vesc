#include "goat_vesc/vesc_client.hpp"

#include <algorithm>
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
    std::cout
        << "Usage: " << argv0 << " [device_path] [baud] [center] [amplitude]\n"
        << "  device_path: serial device such as /dev/ttyACM0\n"
        << "  baud: supported baud rate, default 115200\n"
        << "  center: servo neutral position, default 0.50\n"
        << "  amplitude: sweep half-range around center, default 0.15\n";
}

float clamp_servo(float value) {
    return std::max(0.0f, std::min(1.0f, value));
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
        float center = 0.50f;
        float amplitude = 0.15f;

        if (argc > 1) {
            config.device_path = argv[1];
        }
        if (argc > 2) {
            config.baud = std::stoi(argv[2]);
        }
        if (argc > 3) {
            center = std::stof(argv[3]);
        }
        if (argc > 4) {
            amplitude = std::stof(argv[4]);
        }

        center = clamp_servo(center);
        amplitude = std::max(0.0f, amplitude);

        config.imu_poll_interval = 0ms;
        config.motor_poll_interval = 50ms;
        config.poll_response_timeout = 100ms;

        const float left = clamp_servo(center - amplitude);
        const float right = clamp_servo(center + amplitude);

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
        std::cout << "Servo center=" << center
                  << " left=" << left
                  << " right=" << right << '\n';

        if (auto fw = client.request_fw_version(300ms)) {
            std::cout << "Firmware: "
                      << static_cast<int>(fw->major) << "."
                      << static_cast<int>(fw->minor) << '\n';
        } else {
            std::cout << "Firmware query timed out\n";
        }

        const std::vector<float> sweep{
            center,
            left,
            center,
            right,
            center,
            left,
            center,
            right,
            center,
        };

        std::cout << "Starting servo sweep\n";
        for (const float position : sweep) {
            std::cout << "  set_servo_pos(" << position << ")\n";
            if (!client.set_servo_pos(position)) {
                std::cerr << "Failed to send servo command\n";
                (void)client.set_servo_pos(center);
                client.disconnect();
                return 3;
            }
            std::this_thread::sleep_for(700ms);
        }

        std::cout << "Returning servo to center\n";
        if (!client.set_servo_pos(center)) {
            std::cerr << "Failed to send final center servo command\n";
        }
        std::this_thread::sleep_for(300ms);

        client.disconnect();
        std::cout << "Servo sweep complete\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Servo sweep error: " << ex.what() << '\n';
        return 4;
    }
}
