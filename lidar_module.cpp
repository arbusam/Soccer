/*
 * RPLidar C1 Python Module (pybind11)
 * 
 * Exposes LIDAR scanning functionality to Python.
 * Runs a background thread to continuously capture scan data.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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

void scan_thread_func() {
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
                
                int quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
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

// NOTE: Call this earlier as the lidar gets more accurate after spinning for a while.
bool init_lidar(const std::string& port, int baudrate) {
    if (g_driver != nullptr) {
        throw std::runtime_error("LIDAR already initialized. Call shutdown_lidar() first.");
    }
    
    // Create driver
    g_driver = *createLidarDriver();
    if (!g_driver) {
        throw std::runtime_error("Failed to create LIDAR driver (insufficient memory)");
    }
    
    try {
        g_channel = *createSerialPortChannel(port.c_str(), baudrate);
        if (!g_channel) {
            throw std::runtime_error("Failed to create LIDAR channel");
        }
        
        if (SL_IS_FAIL(g_driver->connect(g_channel))) {
            throw std::runtime_error("Failed to connect to LIDAR at " + port);
        }
        
        sl_lidar_response_device_info_t devinfo;
        sl_result op_result = g_driver->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            throw std::runtime_error("Failed to get LIDAR device info");
        }
        
        sl_lidar_response_device_health_t healthinfo;
        op_result = g_driver->getHealth(healthinfo);
        if (SL_IS_FAIL(op_result) || healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            throw std::runtime_error("LIDAR health check failed");
        }
        
        // Start motor
        g_driver->setMotorSpeed();
        g_driver->startScan(0, 1);
    } catch (...) {
        release_driver_resources();
        throw;
    }
    
    g_running.store(true);
    g_scan_thread = std::thread(scan_thread_func);
    
    printf("LIDAR initialized successfully on %s at %d baud\n", port.c_str(), baudrate);
    return true;
}

void shutdown_lidar() {
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
    
    printf("LIDAR shutdown complete\n");
}

bool is_initialized() {
    return g_driver != nullptr && g_running.load();
}

bool is_scan_ready() {
    return g_scan_ready.load();
}

// Get the latest scan as a numpy array (Nx3: angle_deg, distance_mm, quality)
py::array_t<float> get_scan_numpy() {
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

// Output: angle_deg, distance_mm, quality
py::list get_scan_list() {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    
    py::list result;
    for (const auto& pt : g_latest_scan) {
        result.append(py::make_tuple(pt.angle_deg, pt.distance_mm, pt.quality));
    }
    return result;
}

float get_distance_at_angle(float target_angle) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    
    if (g_latest_scan.empty()) {
        return -1.0f;
    }
    
    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle < 0) target_angle += 360.0f;
    
    float best_distance = -1.0f;
    float min_angle_diff = 360.0f;
    
    for (const auto& pt : g_latest_scan) {
        float angle = fmodf(pt.angle_deg, 360.0f);
        if (angle < 0) angle += 360.0f;
        
        float diff = fabs(angle - target_angle);
        if (diff > 180.0f) diff = 360.0f - diff;
        
        if (diff < min_angle_diff) {
            min_angle_diff = diff;
            best_distance = pt.distance_mm;
        }
    }
    
    return best_distance;
}

py::list get_sector_distances(int num_sectors) {
    if (num_sectors <= 0) {
        throw std::invalid_argument("num_sectors must be positive");
    }

    std::lock_guard<std::mutex> lock(g_data_mutex);
    
    float sector_size = 360.0f / num_sectors;
    std::vector<float> min_distances(num_sectors, -1.0f);
    
    for (const auto& pt : g_latest_scan) {
        float angle = fmodf(pt.angle_deg, 360.0f);
        if (angle < 0) angle += 360.0f;
        
        int sector = (int)(angle / sector_size);
        if (sector >= num_sectors) sector = num_sectors - 1;
        
        if (min_distances[sector] < 0 || pt.distance_mm < min_distances[sector]) {
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

int get_scan_count() {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    return (int)g_latest_scan.size();
}

// Python bindings
PYBIND11_MODULE(lidar, m) {
    m.doc() = "RPLidar C1 Python module - provides real-time LIDAR scan data";
    
    m.def("init", &init_lidar, 
          py::arg("port") = "/dev/ttyUSB0",
          py::arg("baudrate") = 460800,
          "Initialize the LIDAR. Call once at startup.");
    
    m.def("shutdown", &shutdown_lidar,
          "Shutdown the LIDAR. Call before exiting.");
    
    m.def("is_initialized", &is_initialized,
          "Check if LIDAR is initialized and running.");
    
    m.def("is_scan_ready", &is_scan_ready,
          "Check if at least one scan has been captured.");
    
    m.def("get_scan_numpy", &get_scan_numpy,
          "Get latest scan as numpy array (Nx3: angle_deg, distance_mm, quality)");
    
    m.def("get_scan_list", &get_scan_list,
          "Get latest scan as list of (angle_deg, distance_mm, quality) tuples");
    
    m.def("get_distance_at_angle", &get_distance_at_angle,
          py::arg("angle"),
          "Get distance (mm) at closest angle to target. Returns -1 if no reading.");
    
    m.def("get_sector_distances", &get_sector_distances,
          py::arg("num_sectors") = 8,
          "Get minimum distances in angular sectors. Returns list of (center_angle, min_distance).");
    
    m.def("get_scan_count", &get_scan_count,
          "Get number of points in latest scan.");
}

