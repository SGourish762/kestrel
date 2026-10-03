/*
 * fusion.h - 2-state Kalman filter fusing IMU, wheel odometry and GPS.
 *
 *   state x = [position, velocity]
 *   predict: driven by IMU longitudinal acceleration (control input), 200 Hz
 *   update:  wheel-speed velocity (100 Hz) and GPS position (10 Hz)
 *
 * Each update is gated on the normalized innovation squared (NIS): a
 * measurement whose innovation is implausible given the filter's own
 * covariance is rejected instead of corrupting the estimate. This is how
 * production estimators survive wheel slip and GPS multipath.
 */
#ifndef KESTREL_FUSION_H
#define KESTREL_FUSION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double x[2];    /* position (m), velocity (m/s) */
    double P[2][2]; /* covariance */
    double q_acc;   /* accel process noise std (m/s^2) */
    double r_vel;   /* wheel velocity noise std (m/s) */
    double r_pos;   /* GPS position noise std (m) */
    double gate;    /* NIS rejection threshold (chi^2, 1 dof) */
    bool initialized;
    uint32_t rejected_vel, rejected_pos;
    uint32_t accepted_vel, accepted_pos;
} kf1d;

void kf_init(kf1d *kf, double q_acc, double r_vel, double r_pos, double gate);
void kf_predict(kf1d *kf, double accel, double dt);
/* Return true if the measurement was accepted. */
bool kf_update_vel(kf1d *kf, double v_meas);
bool kf_update_pos(kf1d *kf, double p_meas);

/* Robust vehicle speed from 4 wheel speeds: mean of the middle two, which
 * discards a single slipping or locked wheel. Input/output in the same unit. */
double kf_wheel_speed(const double w[4]);

#endif
