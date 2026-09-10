// Include the implementation to exercise actual particle propagation without
// exposing particle mutation hooks in the public Python API.
#include "../lib/localisation.cpp"
#include "../lib/scan_timing.h"
#include <cassert>
#include <limits>

static double measured_variance(float speed, int steps) {
    g_rng.seed(123);
    double sum = 0.0, sum_sq = 0.0;
    const int samples = 20000;
    for (int i = 0; i < samples; ++i) {
        Particle p = {1000.0f, 900.0f, 0.0f, 1.0f};
        for (int j = 0; j < steps; ++j) {
            // Hold heading fixed so this measures translation uncertainty.
            p.yaw_deg = 0.0f;
            propagate_particle(p, speed, 0.0f, 0.0f, 0.1f / steps, true);
        }
        const double error = p.x - 1000.0 - speed * 0.1;
        sum += error;
        sum_sq += error * error;
    }
    assert(std::abs(sum / samples) < 1.5);
    return sum_sq / samples - (sum / samples) * (sum / samples);
}

int main() {
    // 1001 samples at 100 us span 100 ms, regardless of retrieval delay.
    assert(std::abs(scan_midpoint_s(1000000, 1001, 100.0, 1.2) - 1.05) < 1e-9);
    assert(std::abs(scan_midpoint_s(1000000, 1001, 100.0, 1.5) - 1.05) < 1e-9);
    assert(scan_midpoint_s(0, 1001, 100.0, 1.2) < 0);
    assert(scan_midpoint_s(1000000, 0, 100.0, 1.2) < 0);
    assert(scan_midpoint_s(1000000, 1001, 0.0, 1.2) < 0);
    assert(scan_midpoint_s(1000000, 1001, 100.0, 1.0) < 0);
    assert(scan_midpoint_s(1000000, 1001,
                          std::numeric_limits<double>::quiet_NaN(), 1.2) < 0);

    for (float speed : {0.0f, 200.0f, 500.0f}) {
        const double expected = (64.0 + std::pow(0.30 * speed, 2)) * 0.1;
        for (int steps : {1, 5, 10}) {
            assert(std::abs(measured_variance(speed, steps) / expected - 1.0) < 0.05);
        }
    }
    loc_set_motion_noise(0.0f);
    assert(std::abs(measured_variance(500.0f, 5) / 6.4 - 1.0) < 0.05);
    for (float invalid : {-1.0f, std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()}) {
        bool threw = false;
        try { loc_set_motion_noise(invalid); }
        catch (const std::invalid_argument&) { threw = true; }
        assert(threw);
    }
    loc_set_motion_noise(0.30f);
    Particle p = {1000.0f, 900.0f, 0.0f, 1.0f};
    const auto rng_before = g_rng;
    propagate_particle(p, 500.0f, 0.0f, 0.0f, 0.1f, false);
    assert(p.x == 1050.0f && p.y == 900.0f && p.yaw_deg == 0.0f);
    assert(g_rng == rng_before); // Scan replay must not add noise again.
}
