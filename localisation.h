#ifndef LOCALISATION_H
#define LOCALISATION_H

struct LocScanPoint {
    float angle_deg;
    float distance_mm;
    int quality;
};

struct LocPose {
    float x;
    float y;
    float yaw_deg;
    float confidence;
    bool ok;
};

void loc_init_map(float pitch_x, float pitch_y);
void loc_start();
void loc_stop();
void loc_reset();

void loc_predict_odometry(float vx_mm_s, float vy_mm_s, float omega_deg_s, float dt_s);

void loc_update_scan(const LocScanPoint* points, int count,
                     float min_range_mm, float max_range_mm, int min_quality);

bool loc_scan_updates_allowed();
bool loc_is_ready();
LocPose loc_get_pose();

#endif
