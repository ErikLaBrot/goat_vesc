#include "test_support.hpp"

#include "goat_vesc/packet_parser.hpp"
#include "goat_vesc/protocol_ids.hpp"
#include "goat_vesc/vesc_client.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

using namespace goat_vesc;
using namespace std::chrono_literals;
using namespace test_support;

std::vector<std::uint8_t> make_fw_response() {
    return frame_payload({
        static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
        6,
        5,
    });
}

std::vector<std::uint8_t> make_imu_response(float seed) {
    std::vector<std::uint8_t> payload;
    append_u8(payload, static_cast<std::uint8_t>(VescPacketCommID::GetImuData));
    append_u16(payload, DefaultVescImuMask);
    for (int i = 0; i < 16; ++i) {
        append_f32(payload, seed + static_cast<float>(i) * 0.25f);
    }
    return frame_payload(payload);
}

std::vector<std::uint8_t> make_values_response(std::int32_t rpm) {
    std::vector<std::uint8_t> payload;
    append_u8(payload, static_cast<std::uint8_t>(VescPacketCommID::GetValues));
    append_i16(payload, 325);
    append_i16(payload, 301);
    append_i32(payload, 1234);
    append_i32(payload, 890);
    append_i32(payload, 0);
    append_i32(payload, 0);
    append_i16(payload, 455);
    append_i32(payload, rpm);
    append_i16(payload, 523);
    append_i32(payload, 0);
    append_i32(payload, 0);
    append_i32(payload, 0);
    append_i32(payload, 0);
    append_i32(payload, 77);
    append_i32(payload, 88);
    append_u8(payload, 0);
    return frame_payload(payload);
}

std::int16_t read_i16(const std::vector<std::uint8_t>& payload, std::size_t offset) {
    assert(offset + 1 < payload.size());
    const auto hi = static_cast<std::uint16_t>(payload[offset]);
    const auto lo = static_cast<std::uint16_t>(payload[offset + 1]);
    return static_cast<std::int16_t>((hi << 8) | lo);
}

std::int32_t read_i32(const std::vector<std::uint8_t>& payload, std::size_t offset) {
    assert(offset + 3 < payload.size());
    const auto b0 = static_cast<std::uint32_t>(payload[offset]);
    const auto b1 = static_cast<std::uint32_t>(payload[offset + 1]);
    const auto b2 = static_cast<std::uint32_t>(payload[offset + 2]);
    const auto b3 = static_cast<std::uint32_t>(payload[offset + 3]);
    return static_cast<std::int32_t>((b0 << 24) | (b1 << 16) | (b2 << 8) | b3);
}

void write_all(int fd, const std::vector<std::uint8_t>& bytes) {
    const std::uint8_t* cursor = bytes.data();
    std::size_t remaining = bytes.size();
    while (remaining > 0) {
        const ssize_t n = ::write(fd, cursor, remaining);
        if (n < 0) {
            throw std::runtime_error("write failed");
        }
        cursor += n;
        remaining -= static_cast<std::size_t>(n);
    }
}

template <typename Predicate>
void wait_until(Predicate predicate, std::chrono::milliseconds timeout, const char* message) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error(message);
        }
        std::this_thread::sleep_for(5ms);
    }
}

template <typename Loader>
int wait_for_quiescence(
    Loader load,
    std::chrono::milliseconds stable_for,
    std::chrono::milliseconds timeout,
    const char* message) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    int last_value = load();
    auto stable_since = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        const int current = load();
        if (current != last_value) {
            last_value = current;
            stable_since = std::chrono::steady_clock::now();
        } else if (std::chrono::steady_clock::now() - stable_since >= stable_for) {
            return current;
        }
        std::this_thread::sleep_for(5ms);
    }

    throw std::runtime_error(message);
}

struct FakeVesc {
    explicit FakeVesc(bool drop_first_imu_reply = false, bool respond_to_fw = true)
        : drop_first_imu(drop_first_imu_reply), respond_fw(respond_to_fw) {
        int fds[2]{-1, -1};
        if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
            throw std::runtime_error("socketpair failed");
        }
        server_fd = fds[0];
        client_fd = fds[1];
        const int flags = ::fcntl(server_fd, F_GETFL, 0);
        (void)::fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);
        worker = std::thread([this] { run(); });
    }

    ~FakeVesc() {
        running.store(false);
        if (worker.joinable()) {
            worker.join();
        }
        if (server_fd >= 0) {
            ::close(server_fd);
        }
        if (client_fd >= 0) {
            ::close(client_fd);
        }
    }

    bool open_client_fd(int& fd_out) {
        if (client_fd < 0) {
            return false;
        }
        fd_out = client_fd;
        client_fd = -1;
        return true;
    }

    std::atomic<int> imu_requests{0};
    std::atomic<int> value_requests{0};
    std::atomic<int> rpm_commands{0};
    std::atomic<int> current_commands{0};
    std::atomic<int> duty_commands{0};
    std::atomic<int> brake_current_commands{0};
    std::atomic<int> servo_commands{0};
    std::atomic<std::int32_t> last_duty_raw{0};
    std::atomic<std::int32_t> last_brake_current_raw{0};
    std::atomic<std::int16_t> last_servo_raw{0};

private:
    void run() {
        VescPacketParser parser;
        float imu_seed = 1.0f;
        std::int32_t rpm_value = 1400;

        while (running.load()) {
            std::uint8_t buffer[512];
            const ssize_t n = ::read(server_fd, buffer, sizeof(buffer));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    if (auto payload = parser.feed_byte(buffer[static_cast<std::size_t>(i)])) {
                        const auto id = payload->front();
                        if (id == static_cast<std::uint8_t>(VescPacketCommID::GetImuData)) {
                            const int count = ++imu_requests;
                            if (drop_first_imu && count == 1) {
                                continue;
                            }
                            write_all(server_fd, make_imu_response(imu_seed));
                            imu_seed += 1.0f;
                        } else if (id == static_cast<std::uint8_t>(VescPacketCommID::GetValues)) {
                            ++value_requests;
                            write_all(server_fd, make_values_response(rpm_value));
                            rpm_value += 25;
                        } else if (id == static_cast<std::uint8_t>(VescPacketCommID::FwVersion)) {
                            if (!respond_fw) {
                                continue;
                            }
                            write_all(server_fd, make_fw_response());
                        } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetRpm)) {
                            ++rpm_commands;
                        } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetCurrent)) {
                            ++current_commands;
                        } else if (
                            id == static_cast<std::uint8_t>(VescPacketCommID::SetDuty) &&
                            payload->size() == 5) {
                            last_duty_raw.store(read_i32(*payload, 1));
                            ++duty_commands;
                        } else if (
                            id == static_cast<std::uint8_t>(VescPacketCommID::SetCurrentBrake) &&
                            payload->size() == 5) {
                            last_brake_current_raw.store(read_i32(*payload, 1));
                            ++brake_current_commands;
                        } else if (
                            id == static_cast<std::uint8_t>(VescPacketCommID::SetServoPos) &&
                            payload->size() == 3) {
                            last_servo_raw.store(read_i16(*payload, 1));
                            ++servo_commands;
                        }
                    }
                }
                continue;
            }

            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                std::this_thread::sleep_for(2ms);
                continue;
            }
            if (n < 0 && errno == EINTR) {
                continue;
            }
            std::this_thread::sleep_for(2ms);
        }
    }

    int server_fd{-1};
    int client_fd{-1};
    std::atomic<bool> running{true};
    bool drop_first_imu{false};
    bool respond_fw{true};
    std::thread worker;
};

void test_client_polling_and_subscriptions() {
    FakeVesc fake;
    std::atomic<std::uint64_t> stamp_counter{1000};

    VescConfig config;
    config.imu_poll_interval = 20ms;
    config.motor_poll_interval = 40ms;
    config.poll_response_timeout = 15ms;
    config.query_guard_window = 5ms;
    config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(1000); };
    config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);
    std::atomic<int> imu_callbacks{0};
    std::atomic<int> current_from_callback{0};

    auto imu_handle = client.subscribe_imu([&](const VescIMUData& data) {
        ++imu_callbacks;
        assert(data.stamp_ns >= 1000);
        (void)client.latest_imu();
        if (current_from_callback.fetch_add(1) == 0) {
            assert(client.set_current(1.5f));
        }
    });

    auto motor_handle = client.subscribe_motor_state([&](const VescMotorState& state) {
        assert(state.stamp_ns >= 1000);
        assert(state.vin > 0.0f);
    });

    assert(client.connect());

    wait_until([&] { return fake.imu_requests.load() >= 1; }, 500ms, "imu poll requests were not sent");
    {
        const auto deadline = std::chrono::steady_clock::now() + 500ms;
        while (imu_callbacks.load() < 2) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error(
                    "imu callbacks did not arrive; requests=" + std::to_string(fake.imu_requests.load()) +
                    " latest_imu=" + std::to_string(client.latest_imu().has_value()) +
                    " value_requests=" + std::to_string(fake.value_requests.load()) +
                    " latest_motor=" + std::to_string(client.latest_motor_state().has_value()));
            }
            std::this_thread::sleep_for(5ms);
        }
    }
    wait_until([&] {
        return client.latest_motor_state().has_value();
    }, 500ms, "motor state did not arrive");

    const auto imu = client.latest_imu();
    const auto motor = client.latest_motor_state();
    assert(imu.has_value());
    assert(motor.has_value());
    assert(imu->stamp_ns > 0);
    assert(motor->stamp_ns > 0);
    assert(fake.current_commands.load() >= 1);

    const auto before = imu_callbacks.load();
    const auto version = client.request_fw_version(200ms);
    assert(version.has_value());
    assert(version->major == 6);
    assert(version->minor == 5);
    wait_until([&] { return imu_callbacks.load() > before; }, 500ms, "imu polling stalled after fw query");

    std::vector<std::thread> writers;
    for (int i = 0; i < 4; ++i) {
        writers.emplace_back([&client, i] {
            for (int j = 0; j < 10; ++j) {
                assert(client.set_rpm(1000 + i * 100 + j));
            }
        });
    }
    for (auto& writer : writers) {
        writer.join();
    }

    wait_until([&] { return fake.rpm_commands.load() >= 40; }, 500ms, "rpm commands were dropped");
    client.disconnect();

    (void)imu_handle;
    (void)motor_handle;
}

void test_poll_timeout_recovers() {
    FakeVesc fake(true);
    VescConfig config;
    config.imu_poll_interval = 20ms;
    config.motor_poll_interval = 30ms;
    config.poll_response_timeout = 15ms;
    config.query_guard_window = 5ms;
    config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);
    assert(client.connect());

    wait_until([&] { return fake.imu_requests.load() >= 2; }, 500ms, "imu poll did not retry after timeout");
    wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache never recovered");
    wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms, "motor polling stalled");
    client.disconnect();
}

void test_control_command_delivery() {
    FakeVesc fake;
    VescConfig config;
    config.imu_poll_interval = 0ms;
    config.motor_poll_interval = 0ms;
    config.poll_response_timeout = 15ms;
    config.query_guard_window = 5ms;
    config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);
    assert(client.connect());

    assert(client.set_duty(0.2f));
    assert(client.set_current_brake(-1.5f));
    assert(client.set_servo_pos(0.5f));

    wait_until([&] { return fake.duty_commands.load() == 1; }, 500ms, "duty command was not delivered");
    wait_until([&] { return fake.brake_current_commands.load() == 1; }, 500ms,
        "brake current command was not delivered");
    wait_until([&] { return fake.servo_commands.load() == 1; }, 500ms, "servo command was not delivered");

    assert(fake.last_duty_raw.load() == 20000);
    assert(fake.last_brake_current_raw.load() == -1500);
    assert(fake.last_servo_raw.load() == 500);

    client.disconnect();
}

void test_connect_disconnect_edges() {
    FakeVesc fake;
    std::atomic<int> open_calls{0};

    VescConfig config;
    config.imu_poll_interval = 0ms;
    config.motor_poll_interval = 0ms;
    config.poll_response_timeout = 15ms;
    config.query_guard_window = 5ms;
    config.open_serial_fn = [&fake, &open_calls](const VescConfig&, int& fd_out) {
        ++open_calls;
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);

    client.disconnect();
    assert(!client.is_connected());

    assert(client.connect());
    assert(client.is_connected());
    assert(open_calls.load() == 1);

    assert(client.connect());
    assert(client.is_connected());
    assert(open_calls.load() == 1);

    assert(client.set_rpm(1234));
    wait_until([&] { return fake.rpm_commands.load() == 1; }, 500ms, "rpm command was not delivered");

    client.disconnect();
    assert(!client.is_connected());

    client.disconnect();
    assert(!client.is_connected());
    assert(!client.set_rpm(2345));
    assert(!client.request_fw_version(20ms).has_value());
}

void test_runtime_poll_interval_updates() {
    FakeVesc fake;
    std::atomic<std::uint64_t> stamp_counter{1000};

    VescConfig config;
    config.imu_poll_interval = 0ms;
    config.motor_poll_interval = 0ms;
    config.poll_response_timeout = 15ms;
    config.query_guard_window = 5ms;
    config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(1000); };
    config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);
    assert(client.connect());

    std::this_thread::sleep_for(60ms);
    assert(fake.imu_requests.load() == 0);
    assert(fake.value_requests.load() == 0);
    assert(!client.latest_imu().has_value());
    assert(!client.latest_motor_state().has_value());

    client.set_imu_poll_interval(20ms);
    client.set_motor_poll_interval(30ms);

    wait_until([&] { return fake.imu_requests.load() >= 1; }, 500ms, "imu polling did not start");
    wait_until([&] { return fake.value_requests.load() >= 1; }, 500ms, "motor polling did not start");
    wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache did not update");
    wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms, "motor cache did not update");

    const auto first_imu = client.latest_imu();
    const auto first_motor = client.latest_motor_state();
    assert(first_imu.has_value());
    assert(first_motor.has_value());

    client.set_imu_poll_interval(0ms);
    client.set_motor_poll_interval(0ms);

    const int quiet_imu_requests = wait_for_quiescence(
        [&] { return fake.imu_requests.load(); }, 50ms, 300ms, "imu polling never quiesced");
    const int quiet_motor_requests = wait_for_quiescence(
        [&] { return fake.value_requests.load(); }, 50ms, 300ms, "motor polling never quiesced");

    client.set_imu_poll_interval(15ms);
    client.set_motor_poll_interval(15ms);

    wait_until([&] { return fake.imu_requests.load() > quiet_imu_requests; }, 500ms, "imu polling did not restart");
    wait_until(
        [&] { return fake.value_requests.load() > quiet_motor_requests; }, 500ms, "motor polling did not restart");
    wait_until([&] {
        const auto imu = client.latest_imu();
        return imu.has_value() && imu->stamp_ns > first_imu->stamp_ns;
    }, 500ms, "imu cache did not refresh after poll update");
    wait_until([&] {
        const auto motor = client.latest_motor_state();
        return motor.has_value() && motor->stamp_ns > first_motor->stamp_ns;
    }, 500ms, "motor cache did not refresh after poll update");

    client.disconnect();
}

void test_concurrent_poll_updates_keep_client_responsive() {
    FakeVesc fake;
    std::atomic<std::uint64_t> stamp_counter{5000};

    VescConfig config;
    config.imu_poll_interval = 25ms;
    config.motor_poll_interval = 35ms;
    config.poll_response_timeout = 15ms;
    config.query_guard_window = 2ms;
    config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(1000); };
    config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);
    assert(client.connect());

    wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache never populated");
    wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms, "motor cache never populated");

    std::thread updater([&client] {
        const std::array imu_intervals{0ms, 20ms, 45ms, 15ms};
        const std::array motor_intervals{0ms, 30ms, 50ms, 20ms};
        for (int i = 0; i < 24; ++i) {
            client.set_imu_poll_interval(imu_intervals[static_cast<std::size_t>(i) % imu_intervals.size()]);
            client.set_motor_poll_interval(motor_intervals[static_cast<std::size_t>(i) % motor_intervals.size()]);
            std::this_thread::sleep_for(4ms);
        }
        client.set_imu_poll_interval(20ms);
        client.set_motor_poll_interval(20ms);
    });

    std::thread reader([&client] {
        for (int i = 0; i < 200; ++i) {
            (void)client.latest_imu();
            (void)client.latest_motor_state();
            std::this_thread::sleep_for(1ms);
        }
    });

    for (int i = 0; i < 3; ++i) {
        const auto version = client.request_fw_version(250ms);
        assert(version.has_value());
        assert(version->major == 6);
        assert(version->minor == 5);
    }

    updater.join();
    reader.join();

    const auto last_imu = client.latest_imu();
    const auto last_motor = client.latest_motor_state();
    assert(last_imu.has_value());
    assert(last_motor.has_value());

    wait_until([&] {
        const auto imu = client.latest_imu();
        return imu.has_value() && imu->stamp_ns > last_imu->stamp_ns;
    }, 500ms, "imu telemetry stalled after concurrent poll updates");
    wait_until([&] {
        const auto motor = client.latest_motor_state();
        return motor.has_value() && motor->stamp_ns > last_motor->stamp_ns;
    }, 500ms, "motor telemetry stalled after concurrent poll updates");

    client.disconnect();
}

void test_disconnect_unblocks_query() {
    FakeVesc fake(false, false);
    VescConfig config;
    config.imu_poll_interval = 50ms;
    config.motor_poll_interval = 100ms;
    config.poll_response_timeout = 20ms;
    config.query_guard_window = 5ms;
    config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
        return fake.open_client_fd(fd_out);
    };

    VescClient client(config);
    assert(client.connect());

    auto future = std::async(std::launch::async, [&client] {
        return client.request_fw_version(500ms);
    });

    std::this_thread::sleep_for(20ms);
    client.disconnect();
    const auto result = future.get();
    assert(!result.has_value());
}

} // namespace

int main() {
    test_client_polling_and_subscriptions();
    test_poll_timeout_recovers();
    test_control_command_delivery();
    test_connect_disconnect_edges();
    test_runtime_poll_interval_updates();
    test_concurrent_poll_updates_keep_client_responsive();
    test_disconnect_unblocks_query();
    return 0;
}
