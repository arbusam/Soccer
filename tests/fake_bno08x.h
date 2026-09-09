#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

// Emulates the device's SHTP advertisements and I2C header repetition. The real
// SH-2/SHTP C code still parses every packet and generates the feature commands.
// The containing FakeWire serializes access to this state for the tests.
struct FakeBno08x {
    using Bytes = std::vector<uint8_t>;
    std::deque<Bytes> packets;
    std::vector<Bytes> writes;
    std::vector<size_t> reads;
    std::array<uint8_t, 8> sequence{};
    size_t cursor = 0;
    bool peeked = false, auto_reports = false, yaw_enabled = false, gyro_enabled = false;
    bool fail_read = false, fail_payload = false, fail_feature = false, omit_product = false;
    bool omit_reset = false;
    double yaw = 0, gyro = 0; // Degrees and radians/second, respectively.
    std::chrono::steady_clock::time_point next_report{};

    static void tlv(Bytes& into, uint8_t tag, const Bytes& value) {
        into.push_back(tag);
        into.push_back(static_cast<uint8_t>(value.size()));
        into.insert(into.end(), value.begin(), value.end());
    }
    static Bytes string(const char* value) {
        Bytes result(value, value + std::char_traits<char>::length(value));
        result.push_back(0);
        return result;
    }
    void queue(uint8_t channel, const Bytes& payload) {
        const auto size = payload.size() + 4;
        Bytes packet{uint8_t(size), uint8_t(size >> 8), channel, sequence[channel]++};
        packet.insert(packet.end(), payload.begin(), payload.end());
        packets.push_back(std::move(packet));
    }
    void reset() {
        packets.clear();
        cursor = 0;
        peeked = yaw_enabled = gyro_enabled = false;
        Bytes advert{0};
        tlv(advert, 1, {1, 0, 0, 0});
        tlv(advert, 8, string("executable"));
        tlv(advert, 6, {1});
        tlv(advert, 9, string("device"));
        tlv(advert, 1, {2, 0, 0, 0});
        tlv(advert, 8, string("sensorhub"));
        tlv(advert, 6, {2});
        tlv(advert, 9, string("control"));
        tlv(advert, 6, {3});
        tlv(advert, 9, string("inputNormal"));
        tlv(advert, 0x81, {0xf8, 16, 0xfb, 5, 0x08, 12, 0x02, 10});
        queue(0, advert);
        if (!omit_reset) queue(1, {1});
    }
    static void fixed16(Bytes& bytes, double value, double scale) {
        const auto v = static_cast<uint16_t>(static_cast<int16_t>(std::lround(value * scale)));
        bytes.push_back(uint8_t(v));
        bytes.push_back(uint8_t(v >> 8));
    }
    void yaw_report(double degrees) {
        Bytes report{0x08, 0, 3, 0};
        fixed16(report, 0, 16384);
        fixed16(report, 0, 16384);
        fixed16(report, std::sin(degrees * std::acos(-1) / 360), 16384);
        fixed16(report, std::cos(degrees * std::acos(-1) / 360), 16384);
        queue(3, report);
    }
    void gyro_report(double radians_s) {
        Bytes report{0x02, 0, 3, 0};
        fixed16(report, 0, 512);
        fixed16(report, 0, 512);
        fixed16(report, radians_s, 512);
        queue(3, report);
    }
    void transfer(bool reading, uint8_t* data, size_t size) {
        assert(size <= 32);
        if (!reading) {
            writes.emplace_back(data, data + size);
            if (size == 5 && data[2] == 1 && data[4] == 1) {
                reset();
            } else if (data[2] == 2 && data[4] == 0xf9 && !omit_product) {
                Bytes ids(64, 0);
                for (size_t i = 0; i < 4; ++i) { ids[i*16] = 0xf8; ids[i*16+2] = 1; }
                queue(2, ids);
            } else if (data[2] == 2 && data[4] == 0xfd) {
                if (fail_feature) throw std::runtime_error("injected IMU feature write failure");
                assert(size == 21);
                if (data[5] == 0x08) yaw_enabled = true;
                if (data[5] == 0x02) gyro_enabled = true;
            }
            return;
        }
        reads.push_back(size);
        if (fail_read) throw std::runtime_error("injected IMU read failure");
        if (fail_payload && peeked) {
            fail_payload = false;
            packets.pop_front();
            peeked = false;
            cursor = 0;
            throw std::runtime_error("injected short IMU read");
        }
        if (packets.empty() && auto_reports && std::chrono::steady_clock::now() >= next_report) {
            if (yaw_enabled) yaw_report(yaw);
            if (gyro_enabled) gyro_report(gyro);
            next_report = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
        }
        std::fill(data, data + size, 0);
        if (packets.empty()) return;
        const auto& packet = packets.front();
        if (!peeked) {
            assert(size == 4);
            std::copy_n(packet.begin(), 4, data);
            peeked = true;
            return;
        }
        const size_t skip = cursor == 0 ? 0 : 4;
        assert(cursor + size - skip <= packet.size());
        if (skip) std::copy_n(packet.begin(), 4, data);
        std::copy_n(packet.begin() + cursor, size - skip, data + skip);
        cursor += size - skip;
        if (cursor == packet.size()) {
            cursor = 0;
            peeked = false;
            packets.pop_front();
        }
    }
    size_t feature_count(uint8_t sensor, uint32_t interval = 10000) const {
        size_t count = 0;
        for (const auto& write : writes) {
            if (write.size() != 21 || write[4] != 0xfd || write[5] != sensor) continue;
            uint32_t encoded = 0;
            for (int i = 0; i < 4; ++i) encoded |= uint32_t(write[9+i]) << (i*8);
            assert(encoded == interval);
            ++count;
        }
        return count;
    }
};
