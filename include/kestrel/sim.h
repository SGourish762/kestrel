/*
 * sim.h - deterministic vehicle + sensor simulator with fault injection.
 *
 * Ground truth is an analytic speed profile, so the fusion error can be
 * computed exactly at any timestamp. Sensors add bias, Gaussian noise and
 * quantization; the fault injector drops frames, corrupts payloads (caught by
 * CRC), makes a wheel slip and makes GPS jump (both caught by the NIS gate).
 */
#ifndef KESTREL_SIM_H
#define KESTREL_SIM_H

#include <stdbool.h>
#include <stdint.h>

#include "kestrel/can.h"
#include "kestrel/dbc.h"

typedef struct {
    uint64_t rng;
    double imu_bias;
    uint8_t counter[KMSG_COUNT];
    double p_drop, p_corrupt, p_slip, p_gps_glitch;
    /* what was injected, so detection can be scored */
    uint64_t inj_drop[KMSG_COUNT];
    uint64_t inj_corrupt, inj_slip, inj_gps_glitch;
    uint64_t frames;
} ksim;

void ksim_init(ksim *s, uint64_t seed, double fault_rate);
void ksim_truth(double t, double *pos, double *vel, double *acc);
/* Build frame for message `msg` at sim time t_ns. Returns false if the fault
 * injector dropped it (the alive counter still advances, as on a real bus). */
bool ksim_frame(ksim *s, int msg, uint64_t t_ns, kcan_frame *out);

double ksim_uniform(uint64_t *state);
double ksim_gauss(uint64_t *state);

#endif
