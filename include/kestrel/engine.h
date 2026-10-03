/*
 * engine.h - the two-thread real-time pipeline.
 *
 *   [RX thread]  scheduler-driven simulator (or candump replay)
 *        |  stamps rx time, pushes kcan_frame
 *        v
 *   [lock-free SPSC ring]
 *        |
 *        v
 *   [processing thread]  kproc: E2E -> decode -> fusion -> supervision
 *                        records end-to-end latency per frame
 */
#ifndef KESTREL_ENGINE_H
#define KESTREL_ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "kestrel/hist.h"
#include "kestrel/proc.h"
#include "kestrel/sched.h"
#include "kestrel/sim.h"

typedef struct {
    double duration_s;
    double fault_rate;
    uint64_t seed;
    const char *replay_path;
    bool replay_fast;
    bool score_truth;
    const char *record_path;
    bool realtime;
    bool busy_poll; /* consumer never sleeps (dedicated core) */
    bool verbose;
    size_t ring_cap;
    uint64_t spin_ns;
} kengine_cfg;

typedef struct {
    bool sim_mode;
    kproc proc;
    kh_hist latency_ns;  /* rx timestamp -> processing complete */
    kh_hist process_ns;  /* time spent in kproc_handle */
    kt_task tasks[KMSG_COUNT];
    size_t ntasks;
    ksim sim;
    uint64_t pushed, ring_full, max_depth;
    double wall_s;
    double utilization, rm_bound;
    bool rt_ok;
} kengine_report;

void kengine_default_cfg(kengine_cfg *c);
/* `out` is large (histograms); allocate it on the heap. */
int kengine_run(const kengine_cfg *cfg, kengine_report *out);
void kengine_print(const kengine_report *r, FILE *fp);
void kengine_print_json(const kengine_report *r, FILE *fp);

/* Generate `seconds` of simulated bus traffic to a candump log, offline. */
int kengine_generate(const char *path, double seconds, double fault_rate, uint64_t seed);

#endif
