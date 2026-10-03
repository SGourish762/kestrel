#include "kestrel/sim.h"

#include <math.h>
#include <string.h>

#include "kestrel/e2e.h"

#define KS_PI 3.14159265358979323846

static uint64_t xorshift64s(uint64_t *s) {
    uint64_t x = *s;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *s = x;
    return x * 0x2545F4914F6CDD1Dull;
}

double ksim_uniform(uint64_t *s) {
    return (double)(xorshift64s(s) >> 11) * (1.0 / 9007199254740992.0);
}

double ksim_gauss(uint64_t *s) { /* Box-Muller */
    double u1 = ksim_uniform(s), u2 = ksim_uniform(s);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * KS_PI * u2);
}

void ksim_init(ksim *s, uint64_t seed, double fault_rate) {
    memset(s, 0, sizeof *s);
    s->rng = seed ? seed : 0x9E3779B97F4A7C15ull;
    s->imu_bias = 0.03;
    s->p_drop = fault_rate;
    s->p_corrupt = fault_rate;
    s->p_slip = fault_rate * 2.0;
    s->p_gps_glitch = fault_rate * 5.0;
}

/* v(t) = 15 + 10 sin(wt): accelerate/brake cycle between 18 and 90 km/h. */
void ksim_truth(double t, double *pos, double *vel, double *acc) {
    const double w = 2.0 * KS_PI / 20.0;
    if (vel) *vel = 15.0 + 10.0 * sin(w * t);
    if (acc) *acc = 10.0 * w * cos(w * t);
    if (pos) *pos = 15.0 * t + (10.0 / w) * (1.0 - cos(w * t));
}

static void put(kcan_frame *f, int msg, int sig, double phys) {
    const kcan_signal *s = &KDBC_VEHICLE[msg].sigs[sig];
    if (kcan_encode(f->data, s, phys) == 0) return;
    /* out of range: saturate rather than wrap, like a real ECU */
    int64_t hi = s->is_signed ? (int64_t)((1ull << (s->length - 1)) - 1) : (int64_t)((1ull << s->length) - 1);
    int64_t lo = s->is_signed ? -(int64_t)(1ull << (s->length - 1)) : 0;
    int64_t raw = ((phys - s->offset) / s->scale) > 0 ? hi : lo;
    kcan_insert_raw(f->data, s->start_bit, s->length, (kcan_order)s->order, (uint64_t)raw);
}

bool ksim_frame(ksim *s, int msg, uint64_t t_ns, kcan_frame *f) {
    const kdbc_message *m = &KDBC_VEHICLE[msg];
    double t = (double)t_ns / 1e9, p, v, a;
    ksim_truth(t, &p, &v, &a);

    memset(f, 0, sizeof *f);
    f->id = m->id;
    f->dlc = m->dlc;
    f->t_ns = t_ns;

    switch (msg) {
    case KMSG_WHEEL: {
        int slip = -1;
        if (ksim_uniform(&s->rng) < s->p_slip) { slip = (int)(xorshift64s(&s->rng) % 4); s->inj_slip++; }
        for (int i = 0; i < 4; i++) {
            double w = v + 0.15 * ksim_gauss(&s->rng);
            if (i == slip) w *= 1.35;
            put(f, msg, i, w * 3.6);
        }
        break;
    }
    case KMSG_IMU:
        put(f, msg, 0, a + s->imu_bias + 0.05 * ksim_gauss(&s->rng));
        put(f, msg, 1, 0.02 * ksim_gauss(&s->rng));
        put(f, msg, 2, 0.1 * ksim_gauss(&s->rng));
        break;
    case KMSG_BATT: {
        double i_a = 40.0 + 15.0 * a;
        put(f, msg, 0, 396.0 - 0.08 * i_a);
        put(f, msg, 1, i_a);
        put(f, msg, 2, 80.0 - 0.01 * t);
        put(f, msg, 3, 31.0);
        break;
    }
    case KMSG_GPS: {
        double gp = p + 1.0 * ksim_gauss(&s->rng);
        if (ksim_uniform(&s->rng) < s->p_gps_glitch) { gp += 40.0; s->inj_gps_glitch++; }
        put(f, msg, 0, gp);
        put(f, msg, 1, v + 0.1 * ksim_gauss(&s->rng));
        put(f, msg, 2, 9.0);
        break;
    }
    default:
        return false;
    }

    if (m->e2e) ke2e_protect(f, s->counter[msg]++);
    s->frames++;

    if (ksim_uniform(&s->rng) < s->p_drop) {
        s->inj_drop[msg]++;
        return false;
    }
    if (m->e2e && ksim_uniform(&s->rng) < s->p_corrupt) {
        f->data[xorshift64s(&s->rng) % (uint64_t)(m->dlc - 1)] ^= (uint8_t)(1u << (xorshift64s(&s->rng) % 8));
        s->inj_corrupt++;
    }
    return true;
}
