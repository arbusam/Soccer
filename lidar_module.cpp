/*
 * RPLidar C1 Python Module (pybind11)
 * 
 * Exposes LIDAR scanning functionality to Python.
 * Runs a background thread to continuously capture scan data.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <stdexcept>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace py = pybind11;
using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===========================================================================
// Scan data
// ===========================================================================

static ILidarDriver* g_driver = nullptr;
static IChannel* g_channel = nullptr;
static std::atomic<bool> g_running{false};
static std::thread g_scan_thread;
static std::mutex g_data_mutex;

struct ScanPoint {
    float angle_deg;
    float distance_mm;
    int quality;
};

static std::vector<ScanPoint> g_latest_scan;
static std::atomic<bool> g_scan_ready{false};

// ===========================================================================
// Coordinate estimation
// ===========================================================================

static std::atomic<bool> g_coord_running{false};
static std::thread g_coord_thread;
static std::mutex g_coord_mutex;

static float g_pitch_x = 2430.0f;
static float g_pitch_y = 1820.0f;
static std::atomic<float> g_yaw_deg{0.0f};

struct CoordResult {
    float x;
    float y;
    float confidence;
    bool ok;
};

static CoordResult g_latest_coord = {-1, -1, 0, false};
static float g_good_x = -1.0f;
static float g_good_y = -1.0f;
static bool g_has_good_coord = false;
static std::atomic<bool> g_coord_ready{false};

// Only touched by the coordinate thread
static float g_prev_x = -1.0f;
static float g_prev_y = -1.0f;
static bool g_has_prev_pose = false;

// Tuning constants
static constexpr float COORD_EPS       = 1e-9f;
static constexpr float COORD_SIGMA     = 30.0f;   // expected range noise (mm)
static constexpr float COORD_CAUCHY_C  = 2.5f;
static constexpr float MIN_RANGE_MM    = 80.0f;
static constexpr float MAX_RANGE_MM    = 6000.0f;
static constexpr int   MIN_BEAM_QUALITY = 5;
static constexpr int   MIN_BEAM_COUNT  = 30;
static constexpr float CONF_THRESHOLD  = 0.35f;
static constexpr float INLIER_THRESH   = 80.0f;   // mm

// ===========================================================================
// Coordinate estimation — algorithms
// ===========================================================================

static inline float cauchy_loss(float z) {
    float r = z / COORD_CAUCHY_C;
    return std::log(1.0f + r * r);
}

// Predicted range from (x,y) along direction (ux,uy) to rectangle [0,Lx]×[0,Ly].
static inline float predict_range(float x, float y,
                                  float ux, float uy,
                                  float Lx, float Ly) {
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

    return t_min;
}

static float score_pose(float x, float y,
                        const float* ux, const float* uy,
                        const float* r_meas, const float* weights,
                        int n, float Lx, float Ly) {
    float cost = 0.0f;
    for (int i = 0; i < n; i++) {
        float r_pred = predict_range(x, y, ux[i], uy[i], Lx, Ly);
        float e = (r_meas[i] - r_pred) / COORD_SIGMA;
        cost += weights[i] * cauchy_loss(e);
    }
    return cost;
}

struct PoseStats {
    float inlier_ratio;
    float mad_mm;
};

static PoseStats compute_stats(float x, float y,
                               const float* ux, const float* uy,
                               const float* r_meas, int n,
                               float Lx, float Ly) {
    std::vector<float> inlier_errors;
    inlier_errors.reserve(n);
    int inlier_count = 0;

    for (int i = 0; i < n; i++) {
        float r_pred = predict_range(x, y, ux[i], uy[i], Lx, Ly);
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

// Derivative-free coarse-to-fine local search.
static void local_refine(float& x, float& y, float& best_cost,
                         const float* ux, const float* uy,
                         const float* r_meas, const float* weights,
                         int n, float Lx, float Ly,
                         float step0 = 120.0f, int levels = 4) {
    static const float dirs[][2] = {
        {1,0},{-1,0},{0,1},{0,-1},
        {1,1},{1,-1},{-1,1},{-1,-1}
    };

    float step = step0;
    for (int level = 0; level < levels; level++) {
        bool improved = true;
        while (improved) {
            improved = false;
            for (const auto& d : dirs) {
                float xn = std::min(std::max(x + d[0] * step, 0.0f), Lx);
                float yn = std::min(std::max(y + d[1] * step, 0.0f), Ly);
                float c = score_pose(xn, yn, ux, uy, r_meas, weights, n, Lx, Ly);
                if (c < best_cost) {
                    x = xn;
                    y = yn;
                    best_cost = c;
                    improved = true;
                }
            }
        }
        step *= 0.5f;
    }
}

// Coarse grid over the entire pitch, then local refinement of best seed.
static void global_search(float& x, float& y, float& best_cost,
                          const float* ux, const float* uy,
                          const float* r_meas, const float* weights,
                          int n, float Lx, float Ly) {
    const int nx = 9, ny = 7;
    best_cost = 1e30f;
    x = Lx * 0.5f;
    y = Ly * 0.5f;

    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            float cx = Lx * ix / (nx - 1);
            float cy = Ly * iy / (ny - 1);
            float c = score_pose(cx, cy, ux, uy, r_meas, weights, n, Lx, Ly);
            if (c < best_cost) {
                best_cost = c;
                x = cx;
                y = cy;
            }
        }
    }

    local_refine(x, y, best_cost, ux, uy, r_meas, weights, n, Lx, Ly);
}

// ===========================================================================
// Driver helpers
// ===========================================================================

static void release_driver_resources() {
    if (g_driver) {
        delete g_driver;
        g_driver = nullptr;
    }
    if (g_channel) {
        delete g_channel;
        g_channel = nullptr;
    }
}

// ===========================================================================
// Scan thread
// ===========================================================================

static void scan_thread_func() {
    sl_lidar_response_measurement_node_hq_t nodes[8192];

    while (g_running.load()) {
        size_t count = _countof(nodes);
        sl_result op_result = g_driver->grabScanDataHq(nodes, count, 0);

        if (SL_IS_OK(op_result)) {
            g_driver->ascendScanData(nodes, count);

            std::vector<ScanPoint> new_scan;
            new_scan.reserve(count);

            for (size_t i = 0; i < count; i++) {
                if (nodes[i].dist_mm_q2 == 0) continue;

                int quality = nodes[i].quality
                              >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality < 5) continue;

                ScanPoint pt;
                pt.angle_deg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
                pt.distance_mm = nodes[i].dist_mm_q2 / 4.0f;
                pt.quality = quality;
                new_scan.push_back(pt);
            }

            {
                std::lock_guard<std::mutex> lock(g_data_mutex);
                g_latest_scan = std::move(new_scan);
                g_scan_ready.store(true);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ===========================================================================
// Coordinate estimation thread
// ===========================================================================

static void coord_thread_func() {
    std::vector<float> ux, uy, r_meas, weights;

    while (g_coord_running.load()) {
        if (!g_scan_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Snapshot scan data
        std::vector<ScanPoint> scan_copy;
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            scan_copy = g_latest_scan;
        }

        if (scan_copy.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        float yaw = g_yaw_deg.load();
        float psi = yaw * (float)(M_PI / 180.0);

        // Convert beams to world-frame unit vectors
        ux.clear();
        uy.clear();
        r_meas.clear();
        weights.clear();

        for (const auto& pt : scan_copy) {
            if (pt.distance_mm < MIN_RANGE_MM ||
                pt.distance_mm > MAX_RANGE_MM)
                continue;
            if (pt.quality < MIN_BEAM_QUALITY) continue;

            float a = pt.angle_deg * (float)(M_PI / 180.0);
            float theta = psi + a;
            ux.push_back(std::cos(theta));
            uy.push_back(std::sin(theta));
            r_meas.push_back(pt.distance_mm);

            float w = (float)(pt.quality - MIN_BEAM_QUALITY) / 30.0f;
            if (w > 1.0f) w = 1.0f;
            weights.push_back(w);
        }

        int n = (int)ux.size();
        if (n < MIN_BEAM_COUNT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        const float* ux_p = ux.data();
        const float* uy_p = uy.data();
        const float* r_p  = r_meas.data();
        const float* w_p  = weights.data();
        float Lx = g_pitch_x, Ly = g_pitch_y;

        float x, y, cost;
        float conf;

        if (g_has_prev_pose) {
            x = g_prev_x;
            y = g_prev_y;
            cost = score_pose(x, y, ux_p, uy_p, r_p, w_p, n, Lx, Ly);
            local_refine(x, y, cost, ux_p, uy_p, r_p, w_p, n, Lx, Ly);

            PoseStats stats = compute_stats(x, y, ux_p, uy_p, r_p, n, Lx, Ly);
            conf = compute_confidence(stats);

            if (conf < CONF_THRESHOLD) {
                global_search(x, y, cost, ux_p, uy_p, r_p, w_p, n, Lx, Ly);
                stats = compute_stats(x, y, ux_p, uy_p, r_p, n, Lx, Ly);
                conf = compute_confidence(stats);
            }
        } else {
            global_search(x, y, cost, ux_p, uy_p, r_p, w_p, n, Lx, Ly);
            PoseStats stats = compute_stats(x, y, ux_p, uy_p, r_p, n, Lx, Ly);
            conf = compute_confidence(stats);
        }

        bool ok = conf >= CONF_THRESHOLD;
        if (ok) {
            g_prev_x = x;
            g_prev_y = y;
            g_has_prev_pose = true;
        }

        {
            std::lock_guard<std::mutex> lock(g_coord_mutex);
            g_latest_coord = {x, y, conf, ok};
            if (ok) {
                g_good_x = x;
                g_good_y = y;
                g_has_good_coord = true;
            }
            g_coord_ready.store(true);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ===========================================================================
// Init / shutdown
// ===========================================================================

// NOTE: Call this early — the lidar gets more accurate after spinning a while.
static bool init_lidar(const std::string& port, int baudrate) {
    if (g_driver != nullptr) {
        throw std::runtime_error(
            "LIDAR already initialized. Call shutdown() first.");
    }

    g_driver = *createLidarDriver();
    if (!g_driver) {
        throw std::runtime_error(
            "Failed to create LIDAR driver (insufficient memory)");
    }

    try {
        g_channel = *createSerialPortChannel(port.c_str(), baudrate);
        if (!g_channel) {
            throw std::runtime_error("Failed to create LIDAR channel");
        }

        if (SL_IS_FAIL(g_driver->connect(g_channel))) {
            throw std::runtime_error(
                "Failed to connect to LIDAR at " + port);
        }

        sl_lidar_response_device_info_t devinfo;
        sl_result op_result = g_driver->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            throw std::runtime_error("Failed to get LIDAR device info");
        }

        sl_lidar_response_device_health_t healthinfo;
        op_result = g_driver->getHealth(healthinfo);
        if (SL_IS_FAIL(op_result) ||
            healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            throw std::runtime_error("LIDAR health check failed");
        }

        g_driver->setMotorSpeed();
        g_driver->startScan(0, 1);
    } catch (...) {
        release_driver_resources();
        throw;
    }

    g_running.store(true);
    g_scan_thread = std::thread(scan_thread_func);

    printf("LIDAR initialized successfully on %s at %d baud\n",
           port.c_str(), baudrate);
    return true;
}

static void shutdown_lidar() {
    // Stop coordinate thread first (it reads scan data).
    if (g_coord_running.load()) {
        g_coord_running.store(false);
        if (g_coord_thread.joinable()) {
            g_coord_thread.join();
        }
    }

    if (g_driver == nullptr) {
        return;
    }

    g_running.store(false);
    if (g_scan_thread.joinable()) {
        g_scan_thread.join();
    }

    g_driver->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    g_driver->setMotorSpeed(0);

    release_driver_resources();

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        g_latest_scan.clear();
        g_scan_ready.store(false);
    }

    {
        std::lock_guard<std::mutex> lock(g_coord_mutex);
        g_latest_coord = {-1, -1, 0, false};
        g_has_good_coord = false;
        g_coord_ready.store(false);
    }
    g_has_prev_pose = false;

    printf("LIDAR shutdown complete\n");
}

// ===========================================================================
// Scan query functions
// ===========================================================================

static bool is_initialized() {
    return g_driver != nullptr && g_running.load();
}

static bool is_scan_ready() {
    return g_scan_ready.load();
}

static py::array_t<float> get_scan_numpy() {
    std::lock_guard<std::mutex> lock(g_data_mutex);

    if (g_latest_scan.empty()) {
        std::vector<ssize_t> empty_shape = {0, 3};
        return py::array_t<float>(empty_shape);
    }

    size_t n = g_latest_scan.size();
    std::vector<ssize_t> shape = {(ssize_t)n, 3};
    py::array_t<float> result(shape);
    auto buf = result.mutable_unchecked<2>();

    for (size_t i = 0; i < n; i++) {
        buf(i, 0) = g_latest_scan[i].angle_deg;
        buf(i, 1) = g_latest_scan[i].distance_mm;
        buf(i, 2) = (float)g_latest_scan[i].quality;
    }

    return result;
}

static py::list get_scan_list() {
    std::lock_guard<std::mutex> lock(g_data_mutex);

    py::list result;
    for (const auto& pt : g_latest_scan) {
        result.append(
            py::make_tuple(pt.angle_deg, pt.distance_mm, pt.quality));
    }
    return result;
}

static float get_distance_at_angle(float target_angle) {
    std::lock_guard<std::mutex> lock(g_data_mutex);

    if (g_latest_scan.empty()) return -1.0f;

    target_angle = std::fmod(target_angle, 360.0f);
    if (target_angle < 0) target_angle += 360.0f;

    float best_distance = -1.0f;
    float min_angle_diff = 360.0f;

    for (const auto& pt : g_latest_scan) {
        float angle = std::fmod(pt.angle_deg, 360.0f);
        if (angle < 0) angle += 360.0f;

        float diff = std::fabs(angle - target_angle);
        if (diff > 180.0f) diff = 360.0f - diff;

        if (diff < min_angle_diff) {
            min_angle_diff = diff;
            best_distance = pt.distance_mm;
        }
    }

    return best_distance;
}

static py::list get_sector_distances(int num_sectors) {
    if (num_sectors <= 0)
        throw std::invalid_argument("num_sectors must be positive");

    std::lock_guard<std::mutex> lock(g_data_mutex);

    float sector_size = 360.0f / num_sectors;
    std::vector<float> min_distances(num_sectors, -1.0f);

    for (const auto& pt : g_latest_scan) {
        float angle = std::fmod(pt.angle_deg, 360.0f);
        if (angle < 0) angle += 360.0f;

        int sector = (int)(angle / sector_size);
        if (sector >= num_sectors) sector = num_sectors - 1;

        if (min_distances[sector] < 0 ||
            pt.distance_mm < min_distances[sector]) {
            min_distances[sector] = pt.distance_mm;
        }
    }

    py::list result;
    for (int i = 0; i < num_sectors; i++) {
        float center_angle = (i + 0.5f) * sector_size;
        result.append(py::make_tuple(center_angle, min_distances[i]));
    }
    return result;
}

static int get_scan_count() {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    return (int)g_latest_scan.size();
}

// ===========================================================================
// Coordinate estimation — Python API
// ===========================================================================

static void start_coordinates(float pitch_x, float pitch_y) {
    if (g_coord_running.load()) {
        throw std::runtime_error(
            "Coordinate estimation already running.");
    }
    g_pitch_x = pitch_x;
    g_pitch_y = pitch_y;
    g_has_prev_pose = false;
    g_has_good_coord = false;
    g_coord_ready.store(false);

    g_coord_running.store(true);
    g_coord_thread = std::thread(coord_thread_func);
    printf("Coordinate estimation started (pitch %.0f x %.0f mm)\n",
           pitch_x, pitch_y);
}

static void set_yaw(float yaw_deg) {
    g_yaw_deg.store(yaw_deg);
}

// Returns (x, y) of last confident estimate, or (None, None).
static py::tuple get_coordinates_py() {
    std::lock_guard<std::mutex> lock(g_coord_mutex);
    if (!g_has_good_coord) {
        return py::make_tuple(py::none(), py::none());
    }
    return py::make_tuple(g_good_x, g_good_y);
}

// Returns (x, y, confidence, ok) — latest raw result for diagnostics.
static py::tuple get_coordinates_info_py() {
    std::lock_guard<std::mutex> lock(g_coord_mutex);
    if (!g_has_good_coord) {
        return py::make_tuple(py::none(), py::none(),
                              g_latest_coord.confidence,
                              g_latest_coord.ok);
    }
    return py::make_tuple(g_good_x, g_good_y,
                          g_latest_coord.confidence,
                          g_latest_coord.ok);
}

static bool is_coordinates_ready() {
    std::lock_guard<std::mutex> lock(g_coord_mutex);
    return g_has_good_coord;
}

// ===========================================================================
// Python module definition
// ===========================================================================

PYBIND11_MODULE(lidar, m) {
    m.doc() = "RPLidar C1 Python module — real-time scan data and "
              "coordinate estimation";

    // --- lifecycle ---
    m.def("init", &init_lidar,
          py::arg("port") = "/dev/ttyUSB0",
          py::arg("baudrate") = 460800,
          "Initialize the LIDAR. Call once at startup.");

    m.def("shutdown", &shutdown_lidar,
          "Shutdown the LIDAR and coordinate estimation.");

    m.def("is_initialized", &is_initialized,
          "Check if LIDAR is initialized and running.");

    m.def("is_scan_ready", &is_scan_ready,
          "Check if at least one scan has been captured.");

    // --- raw scan access ---
    m.def("get_scan_numpy", &get_scan_numpy,
          "Get latest scan as numpy array (Nx3: angle_deg, distance_mm, "
          "quality).");

    m.def("get_scan_list", &get_scan_list,
          "Get latest scan as list of (angle_deg, distance_mm, quality) "
          "tuples.");

    m.def("get_distance_at_angle", &get_distance_at_angle,
          py::arg("angle"),
          "Get distance (mm) at closest angle to target. Returns -1 if "
          "no reading.");

    m.def("get_sector_distances", &get_sector_distances,
          py::arg("num_sectors") = 8,
          "Get minimum distances in angular sectors.");

    m.def("get_scan_count", &get_scan_count,
          "Get number of points in latest scan.");

    // --- coordinate estimation ---
    m.def("start_coordinates", &start_coordinates,
          py::arg("pitch_x"), py::arg("pitch_y"),
          "Start background coordinate estimation thread. "
          "pitch_x/pitch_y are field dimensions in mm.");

    m.def("set_yaw", &set_yaw,
          py::arg("yaw_deg"),
          "Update robot yaw (degrees, 0 = facing +X). "
          "Called each frame from the control loop.");

    m.def("get_coordinates", &get_coordinates_py,
          "Get (x, y) of the last confident position estimate, "
          "or (None, None) if unavailable.");

    m.def("get_coordinates_info", &get_coordinates_info_py,
          "Get (x, y, confidence, ok) — latest estimate with diagnostics.");

    m.def("is_coordinates_ready", &is_coordinates_ready,
          "True once at least one confident position has been computed.");
}
