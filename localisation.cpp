#include "localisation.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <random>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Segment {
    float x1;
    float y1;
    float x2;
    float y2;
};

struct Particle {
    float x;
    float y;
    float yaw_deg;
    float weight;
};

static std::vector<Segment> g_static_segments;
static std::vector<Particle> g_particles;
static std::mutex g_loc_mutex;
static std::mt19937 g_rng(42);

static float g_pitch_x = 2430.0f;
static float g_pitch_y = 1820.0f;
static LocPose g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
static bool g_started = false;
static bool g_ready = false;

static constexpr float COORD_SIGMA = 30.0f;
static constexpr float COORD_EPS = 1e-9f;
static constexpr float INLIER_THRESH = 80.0f;
static constexpr float CONF_THRESHOLD = 0.35f;
static constexpr int PARTICLE_COUNT = 1000;
static constexpr int MIN_BEAM_COUNT = 30;
static constexpr int RAY_STRIDE = 2;
static constexpr float TRANS_NOISE_MM = 8.0f;
static constexpr float YAW_NOISE_DEG = 2.0f;
static constexpr float RECOVERY_FRACTION = 0.05f;
static constexpr float RECOVERY_WEIGHT_THRESHOLD = 1e-12f;

static constexpr float GOAL_LEFT_BACK_X = 226.0f;
static constexpr float GOAL_RIGHT_BACK_X = 2204.0f;
static constexpr float GOAL_LEFT_FRONT_X = 300.0f;
static constexpr float GOAL_RIGHT_FRONT_X = 2130.0f;
static constexpr float GOAL_TOP_Y = 685.0f;
static constexpr float GOAL_BOTTOM_Y = 1135.0f;
static constexpr float GOAL_BACK_BOTTOM_Y = 1140.0f;

static float wrap_angle_deg(float angle) {
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float rand_uniform(float lo, float hi) {
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(g_rng);
}

static float rand_normal(float stddev) {
    std::normal_distribution<float> dist(0.0f, stddev);
    return dist(g_rng);
}

static inline float ray_segment_intersection_distance(float px, float py,
                                                      float ux, float uy,
                                                      const Segment& seg) {
    float sx = seg.x2 - seg.x1;
    float sy = seg.y2 - seg.y1;

    float denom = ux * sy - uy * sx;
    if (std::fabs(denom) <= COORD_EPS) {
        return 1e30f;
    }

    float qpx = seg.x1 - px;
    float qpy = seg.y1 - py;

    float t = (qpx * sy - qpy * sx) / denom;
    float u = (qpx * uy - qpy * ux) / denom;

    if (t > COORD_EPS && u >= -COORD_EPS && u <= 1.0f + COORD_EPS) {
        return t;
    }
    return 1e30f;
}

static inline float predict_range(float x, float y, float ux, float uy, float Lx, float Ly) {
    float t_min = 1e30f;
    if (ux < -COORD_EPS) {
        float t = -x / ux;
        if (t > 0 && t < t_min) t_min = t;
    } else if (ux > COORD_EPS) {
        float t = (Lx - x) / ux;
        if (t > 0 && t < t_min) t_min = t;
    }

    if (uy < -COORD_EPS) {
        float t = -y / uy;
        if (t > 0 && t < t_min) t_min = t;
    } else if (uy > COORD_EPS) {
        float t = (Ly - y) / uy;
        if (t > 0 && t < t_min) t_min = t;
    }

    for (const auto& seg : g_static_segments) {
        float t = ray_segment_intersection_distance(x, y, ux, uy, seg);
        if (t < t_min) t_min = t;
    }

    return t_min;
}

static float score_pose(float x, float y, float yaw_deg,
                        const float* angle_deg, const float* r_meas,
                        const float* weights, int n, float Lx, float Ly) {
    float psi = yaw_deg * (float)(M_PI / 180.0);
    float log_lik = 0.0f;

    for (int i = 0; i < n; i++) {
        float theta = psi + angle_deg[i] * (float)(M_PI / 180.0);
        float ux = std::cos(theta);
        float uy = std::sin(theta);
        float r_pred = predict_range(x, y, ux, uy, Lx, Ly);
        float e = (r_meas[i] - r_pred) / COORD_SIGMA;
        e = std::max(-3.0f, std::min(3.0f, e));
        log_lik += weights[i] * (-0.5f * e * e);
    }
    return log_lik;
}

struct PoseStats {
    float inlier_ratio;
    float mad_mm;
};

static PoseStats compute_stats(float x, float y, float yaw_deg,
                               const float* angle_deg, const float* r_meas,
                               int n, float Lx, float Ly) {
    float psi = yaw_deg * (float)(M_PI / 180.0);
    std::vector<float> inlier_errors;
    inlier_errors.reserve(n);
    int inlier_count = 0;

    for (int i = 0; i < n; i++) {
        float theta = psi + angle_deg[i] * (float)(M_PI / 180.0);
        float ux = std::cos(theta);
        float uy = std::sin(theta);
        float r_pred = predict_range(x, y, ux, uy, Lx, Ly);
        float e = std::fabs(r_meas[i] - r_pred);
        if (e < INLIER_THRESH) {
            inlier_count++;
            inlier_errors.push_back(e);
        }
    }

    PoseStats stats;
    stats.inlier_ratio = (float)inlier_count / std::max(n, 1);

    if (inlier_errors.empty()) {
        stats.mad_mm = 1e9f;
    } else {
        std::sort(inlier_errors.begin(), inlier_errors.end());
        stats.mad_mm = 1.4826f * inlier_errors[inlier_errors.size() / 2];
    }
    return stats;
}

static float compute_confidence(const PoseStats& s) {
    return 0.7f * s.inlier_ratio + 0.3f * std::exp(-s.mad_mm / 80.0f);
}

static void init_particles_uniform() {
    g_particles.resize(PARTICLE_COUNT);
    const float weight = 1.0f / PARTICLE_COUNT;
    for (auto& particle : g_particles) {
        particle.x = rand_uniform(0.0f, g_pitch_x);
        particle.y = rand_uniform(0.0f, g_pitch_y);
        particle.yaw_deg = rand_uniform(-180.0f, 180.0f);
        particle.weight = weight;
    }
}

static void inject_random_particles(float fraction) {
    int count = std::max(1, (int)std::round(fraction * PARTICLE_COUNT));
    for (int i = 0; i < count; i++) {
        int idx = (int)rand_uniform(0.0f, (float)(PARTICLE_COUNT - 1));
        g_particles[idx].x = rand_uniform(0.0f, g_pitch_x);
        g_particles[idx].y = rand_uniform(0.0f, g_pitch_y);
        g_particles[idx].yaw_deg = rand_uniform(-180.0f, 180.0f);
    }
}

static void resample_particles() {
    std::vector<Particle> new_particles(PARTICLE_COUNT);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f / PARTICLE_COUNT);
    float step = 1.0f / PARTICLE_COUNT;
    float cursor = dist(g_rng);

    int index = 0;
    float cumulative = g_particles[0].weight;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        while (cursor > cumulative && index < PARTICLE_COUNT - 1) {
            index++;
            cumulative += g_particles[index].weight;
        }
        new_particles[i] = g_particles[index];
        new_particles[i].weight = step;
        cursor += step;
    }
    g_particles.swap(new_particles);
}

static LocPose estimate_pose_from_particles(const float* angle_deg,
                                            const float* r_meas,
                                            int n) {
    float sum_w = 0.0f;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;

    for (const auto& particle : g_particles) {
        sum_w += particle.weight;
        mean_x += particle.weight * particle.x;
        mean_y += particle.weight * particle.y;
        float yaw_rad = particle.yaw_deg * (float)(M_PI / 180.0);
        sin_sum += particle.weight * std::sin(yaw_rad);
        cos_sum += particle.weight * std::cos(yaw_rad);
    }

    LocPose pose;
    if (sum_w <= 0.0f) {
        pose.ok = false;
        pose.confidence = 0.0f;
        return pose;
    }

    mean_x /= sum_w;
    mean_y /= sum_w;
    pose.x = mean_x;
    pose.y = mean_y;
    pose.yaw_deg = wrap_angle_deg(std::atan2(sin_sum, cos_sum) * (180.0f / (float)M_PI));

    if (angle_deg != nullptr && r_meas != nullptr && n > 0) {
        PoseStats stats = compute_stats(mean_x, mean_y, pose.yaw_deg,
                                        angle_deg, r_meas, n, g_pitch_x, g_pitch_y);
        pose.confidence = compute_confidence(stats);
        pose.ok = pose.confidence >= CONF_THRESHOLD;
    } else {
        pose.confidence = g_pose.confidence;
        pose.ok = g_pose.ok;
    }
    return pose;
}

void loc_init_map(float pitch_x, float pitch_y) {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_pitch_x = pitch_x;
    g_pitch_y = pitch_y;
    g_static_segments.clear();

    g_static_segments.push_back({0.0f, 0.0f, pitch_x, 0.0f});
    g_static_segments.push_back({pitch_x, 0.0f, pitch_x, pitch_y});
    g_static_segments.push_back({pitch_x, pitch_y, 0.0f, pitch_y});
    g_static_segments.push_back({0.0f, pitch_y, 0.0f, 0.0f});

    g_static_segments.push_back({GOAL_LEFT_FRONT_X, GOAL_TOP_Y, 0.0f, GOAL_TOP_Y});
    g_static_segments.push_back({GOAL_LEFT_BACK_X, GOAL_TOP_Y, GOAL_LEFT_BACK_X, GOAL_BACK_BOTTOM_Y});
    g_static_segments.push_back({0.0f, GOAL_BOTTOM_Y, GOAL_LEFT_FRONT_X, GOAL_BOTTOM_Y});
    g_static_segments.push_back({GOAL_RIGHT_FRONT_X, GOAL_TOP_Y, pitch_x, GOAL_TOP_Y});
    g_static_segments.push_back({GOAL_RIGHT_BACK_X, GOAL_TOP_Y, GOAL_RIGHT_BACK_X, GOAL_BACK_BOTTOM_Y});
    g_static_segments.push_back({pitch_x, GOAL_BOTTOM_Y, GOAL_RIGHT_FRONT_X, GOAL_BOTTOM_Y});
}

void loc_start() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    init_particles_uniform();
    g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
    g_ready = false;
    g_started = true;
}

void loc_stop() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    g_started = false;
    g_ready = false;
    g_particles.clear();
    g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
}

void loc_reset() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    if (!g_started) {
        return;
    }
    init_particles_uniform();
    g_pose = {0.0f, 0.0f, 0.0f, 0.0f, false};
    g_ready = false;
}

void loc_predict_odometry(float vx_mm_s, float vy_mm_s, float omega_deg_s, float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_loc_mutex);
    if (!g_started || g_particles.empty()) {
        return;
    }

    for (auto& particle : g_particles) {
        float yaw_rad = particle.yaw_deg * (float)(M_PI / 180.0);
        float cos_yaw = std::cos(yaw_rad);
        float sin_yaw = std::sin(yaw_rad);
        float dx = (vx_mm_s * cos_yaw - vy_mm_s * sin_yaw) * dt_s;
        float dy = (vx_mm_s * sin_yaw + vy_mm_s * cos_yaw) * dt_s;

        particle.x = std::min(std::max(particle.x + dx + rand_normal(TRANS_NOISE_MM), 0.0f), g_pitch_x);
        particle.y = std::min(std::max(particle.y + dy + rand_normal(TRANS_NOISE_MM), 0.0f), g_pitch_y);
        particle.yaw_deg = wrap_angle_deg(
            particle.yaw_deg + omega_deg_s * dt_s + rand_normal(YAW_NOISE_DEG));
    }

    if (g_ready) {
        LocPose updated = estimate_pose_from_particles(nullptr, nullptr, 0);
        g_pose.x = updated.x;
        g_pose.y = updated.y;
        g_pose.yaw_deg = updated.yaw_deg;
    }
}

void loc_update_scan(const LocScanPoint* points, int count,
                     float min_range_mm, float max_range_mm, int min_quality) {
    std::vector<float> angle_deg;
    std::vector<float> r_meas;
    std::vector<float> weights;
    angle_deg.reserve(count / RAY_STRIDE + 1);
    r_meas.reserve(count / RAY_STRIDE + 1);
    weights.reserve(count / RAY_STRIDE + 1);

    for (int i = 0; i < count; i += RAY_STRIDE) {
        const LocScanPoint& pt = points[i];
        if (pt.distance_mm < min_range_mm || pt.distance_mm > max_range_mm) {
            continue;
        }
        if (pt.quality < min_quality) {
            continue;
        }

        angle_deg.push_back(pt.angle_deg);
        r_meas.push_back(pt.distance_mm);

        float w = (float)(pt.quality - min_quality) / 30.0f;
        if (w > 1.0f) w = 1.0f;
        weights.push_back(w);
    }

    const int n = (int)angle_deg.size();
    if (n < MIN_BEAM_COUNT) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_loc_mutex);
    if (!g_started || g_particles.empty()) {
        return;
    }

    for (auto& particle : g_particles) {
        float log_lik = score_pose(particle.x, particle.y, particle.yaw_deg,
                                   angle_deg.data(), r_meas.data(), weights.data(),
                                   n, g_pitch_x, g_pitch_y);
        particle.weight = std::exp(log_lik);
    }

    float weight_sum = 0.0f;
    for (const auto& particle : g_particles) {
        weight_sum += particle.weight;
    }

    if (weight_sum < RECOVERY_WEIGHT_THRESHOLD) {
        init_particles_uniform();
        weight_sum = 1.0f;
    } else if (weight_sum / PARTICLE_COUNT < RECOVERY_WEIGHT_THRESHOLD) {
        inject_random_particles(RECOVERY_FRACTION);
        weight_sum = 0.0f;
        for (const auto& particle : g_particles) {
            weight_sum += particle.weight;
        }
    }

    for (auto& particle : g_particles) {
        particle.weight /= weight_sum;
    }

    g_pose = estimate_pose_from_particles(angle_deg.data(), r_meas.data(), n);
    if (g_pose.ok) {
        g_ready = true;
    }

    resample_particles();
}

bool loc_is_ready() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return g_ready && g_pose.ok;
}

LocPose loc_get_pose() {
    std::lock_guard<std::mutex> lock(g_loc_mutex);
    return g_pose;
}
