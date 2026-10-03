#include "kestrel/fusion.h"

#include <string.h>

void kf_init(kf1d *kf, double q_acc, double r_vel, double r_pos, double gate) {
    memset(kf, 0, sizeof *kf);
    kf->q_acc = q_acc;
    kf->r_vel = r_vel;
    kf->r_pos = r_pos;
    kf->gate = gate;
    kf->P[0][0] = 1e6; /* unknown until first measurement */
    kf->P[1][1] = 1e6;
}

void kf_predict(kf1d *kf, double a, double dt) {
    if (dt <= 0.0) return;
    double p = kf->x[0], v = kf->x[1];
    kf->x[0] = p + v * dt + 0.5 * a * dt * dt;
    kf->x[1] = v + a * dt;

    /* P = F P F^T + G G^T q^2, with F = [[1,dt],[0,1]], G = [dt^2/2, dt] */
    double P00 = kf->P[0][0], P01 = kf->P[0][1], P10 = kf->P[1][0], P11 = kf->P[1][1];
    double n00 = P00 + dt * (P10 + P01) + dt * dt * P11;
    double n01 = P01 + dt * P11;
    double n11 = P11;
    double q = kf->q_acc * kf->q_acc;
    double g0 = 0.5 * dt * dt, g1 = dt;
    kf->P[0][0] = n00 + g0 * g0 * q;
    kf->P[0][1] = n01 + g0 * g1 * q;
    kf->P[1][0] = kf->P[0][1]; /* keep symmetric */
    kf->P[1][1] = n11 + g1 * g1 * q;
}

/* Scalar measurement of state component k with noise std r. */
static bool update(kf1d *kf, int k, double z, double r, uint32_t *rej, uint32_t *acc) {
    double y = z - kf->x[k];
    double S = kf->P[k][k] + r * r;
    if (kf->initialized && (y * y) / S > kf->gate) {
        (*rej)++;
        return false;
    }
    double K0 = kf->P[0][k] / S, K1 = kf->P[1][k] / S;
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;
    /* P = (I - K H) P, H = e_k */
    double Pk0 = kf->P[k][0], Pk1 = kf->P[k][1];
    kf->P[0][0] -= K0 * Pk0;
    kf->P[0][1] -= K0 * Pk1;
    kf->P[1][1] -= K1 * Pk1;
    kf->P[1][0] = kf->P[0][1];
    (*acc)++;
    return true;
}

bool kf_update_vel(kf1d *kf, double v) {
    bool ok = update(kf, 1, v, kf->r_vel, &kf->rejected_vel, &kf->accepted_vel);
    /* gate only once both states have been observed */
    if (kf->accepted_pos && kf->accepted_vel) kf->initialized = true;
    return ok;
}

bool kf_update_pos(kf1d *kf, double p) {
    bool ok = update(kf, 0, p, kf->r_pos, &kf->rejected_pos, &kf->accepted_pos);
    if (kf->accepted_pos && kf->accepted_vel) kf->initialized = true;
    return ok;
}

double kf_wheel_speed(const double w[4]) {
    double a[4] = { w[0], w[1], w[2], w[3] };
    for (int i = 1; i < 4; i++) { /* 4-element insertion sort */
        double t = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > t) { a[j + 1] = a[j]; j--; }
        a[j + 1] = t;
    }
    return 0.5 * (a[1] + a[2]);
}
