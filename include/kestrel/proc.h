/*
 * proc.h - the per-frame processing stage (consumer side of the pipeline):
 *   lookup -> DLC check -> E2E (CRC + alive counter) -> signal decode ->
 *   signal store -> sensor fusion -> freshness/timeout supervision.
 * Pure computation, no I/O, no allocation: identical code runs in the live
 * engine, the replay tool, the benchmarks and the tests.
 */
#ifndef KESTREL_PROC_H
#define KESTREL_PROC_H

#include <stdbool.h>
#include <stdint.h>

#include "kestrel/can.h"
#include "kestrel/dbc.h"
#include "kestrel/e2e.h"
#include "kestrel/fusion.h"

typedef struct {
    uint64_t frames, accepted, unknown_id, bad_dlc;
    uint64_t crc_errors, repeats, lost_frames;
    uint64_t timeouts;
    uint64_t signals_decoded;
} kproc_stats;

typedef struct {
    double value[KSIG_COUNT];
    uint64_t value_t[KSIG_COUNT];
    ke2e_state e2e[KMSG_COUNT];
    uint64_t last_seen[KMSG_COUNT];
    bool seen[KMSG_COUNT];
    bool timed_out[KMSG_COUNT];
    kf1d kf;
    uint64_t last_imu_t;
    /* truth scoring (simulation only) */
    bool score_truth;
    double se_vel, se_pos, se_vel_raw;
    uint64_t n_scored;
    kproc_stats st;
} kproc;

void kproc_init(kproc *p, bool score_truth);
/* Returns true if the frame was accepted and applied. */
bool kproc_handle(kproc *p, const kcan_frame *f);
/* Flag messages not seen within 3 periods of bus time `now_t`. Edge-triggered. */
void kproc_check_timeouts(kproc *p, uint64_t now_t);

double kproc_rmse_vel(const kproc *p);
double kproc_rmse_pos(const kproc *p);
double kproc_rmse_vel_raw(const kproc *p); /* wheel-only baseline, for comparison */

#endif
