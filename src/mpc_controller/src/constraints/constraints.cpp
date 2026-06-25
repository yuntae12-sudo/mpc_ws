#include "constraints.hpp"
#include "../global/utils.hpp"


void clipSteering(MPCControl& u, double max_steering) {
    u.delta = clip(u.delta, -max_steering, max_steering);
}

void clipAcceleration(MPCControl& u, double a_min, double a_max) {
    u.accel = clip(u.accel, a_min, a_max);
}

void clipControl(MPCControl& u, const MPCParams& p) {
    clipSteering(u, p.steering_max);
    clipAcceleration(u, p.accel_min, p.accel_max);
}

void clipSteeringRate(MPCControl& u_cur, const MPCControl& u_prev,
                      double max_rate, double dt) {
    double max_step = max_rate * dt;
    double diff = u_cur.delta - u_prev.delta;
    diff = clip(diff, -max_step, max_step);
    u_cur.delta = u_prev.delta + diff;
}

bool isControlFeasible(const MPCControl& u, const MPCParams& p) {
    if (std::fabs(u.delta) > p.steering_max + 1e-6) return false;
    if (u.accel < p.accel_min - 1e-6 || u.accel > p.accel_max + 1e-6) return false;
    return true;
}

bool isCollisionDetected(const MPCState& s, const CostmapInfo& costmap,
                          double lethal_threshold) {
    if (!costmap.msg || costmap.resolution <= 1e-9 ||
        costmap.width <= 0 || costmap.height <= 0)
        return false;

    int gx = static_cast<int>(std::floor((s.x - costmap.origin_x) / costmap.resolution));
    int gy = static_cast<int>(std::floor((s.y - costmap.origin_y) / costmap.resolution));
    if (gx < 0 || gx >= costmap.width || gy < 0 || gy >= costmap.height)
        return false;

    int idx = gy * costmap.width + gx;
    if (idx < 0 || idx >= static_cast<int>(costmap.msg->data.size()))
        return false;

    int8_t raw = costmap.msg->data[idx];
    if (raw < 0) return false;            // unknown
    return static_cast<double>(raw) >= lethal_threshold;
}
