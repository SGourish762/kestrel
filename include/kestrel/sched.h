/*
 * sched.h - fixed-priority, rate-monotonic cyclic executive.
 *
 * Periodic tasks are released on absolute deadlines (no cumulative drift).
 * When several tasks are ready the highest-priority one runs first
 * (non-preemptive). For each task the executive measures:
 *   - release jitter: actual start - scheduled release
 *   - execution time
 *   - deadline misses: completion after the next release
 *   - overruns: whole periods skipped because the loop fell behind
 */
#ifndef KESTREL_SCHED_H
#define KESTREL_SCHED_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kestrel/hist.h"

typedef void (*kt_fn)(void *ctx, uint64_t release_ns);

typedef struct {
    const char *name;
    kt_fn fn;
    void *ctx;
    uint64_t period_ns;
    uint64_t offset_ns;   /* phase offset to stagger releases */
    uint64_t wcet_ns;     /* budgeted worst-case execution time (for RM analysis) */
    int priority;         /* lower number = higher priority */
    /* runtime stats */
    uint64_t next_release;
    uint64_t runs, deadline_misses, skipped_periods;
    kh_hist jitter_ns;
    kh_hist exec_ns;
} kt_task;

typedef struct {
    kt_task *tasks;
    size_t ntasks;
    uint64_t spin_ns; /* busy-wait window before each release (jitter vs CPU) */
} kt_sched;

/* Liu & Layland utilization bound for n tasks: n(2^(1/n) - 1). */
double kt_rm_bound(size_t n);
/* Sum of wcet/period. */
double kt_utilization(const kt_task *tasks, size_t n);

void kt_task_init(kt_task *t, const char *name, kt_fn fn, void *ctx,
                  uint64_t period_ns, uint64_t offset_ns, uint64_t wcet_ns, int priority);

/* Run until duration elapses or *stop becomes true. */
void kt_sched_run(kt_sched *s, uint64_t duration_ns, atomic_bool *stop);

#endif
