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
