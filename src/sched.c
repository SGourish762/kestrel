#include "kestrel/sched.h"

#include <math.h>

#include "kestrel/clock.h"

double kt_rm_bound(size_t n) {
    if (n == 0) return 1.0;
    return (double)n * (pow(2.0, 1.0 / (double)n) - 1.0);
}

double kt_utilization(const kt_task *t, size_t n) {
    double u = 0.0;
    for (size_t i = 0; i < n; i++) u += (double)t[i].wcet_ns / (double)t[i].period_ns;
    return u;
}

void kt_task_init(kt_task *t, const char *name, kt_fn fn, void *ctx,
                  uint64_t period_ns, uint64_t offset_ns, uint64_t wcet_ns, int priority) {
    t->name = name;
    t->fn = fn;
    t->ctx = ctx;
    t->period_ns = period_ns;
    t->offset_ns = offset_ns;
    t->wcet_ns = wcet_ns;
    t->priority = priority;
    t->next_release = 0;
    t->runs = t->deadline_misses = t->skipped_periods = 0;
    kh_init(&t->jitter_ns);
    kh_init(&t->exec_ns);
}

void kt_sched_run(kt_sched *s, uint64_t duration_ns, atomic_bool *stop) {
    uint64_t t0 = kc_now_ns() + 20000000; /* start 20 ms out: lets thread start-up/page faults settle */
    uint64_t end = t0 + duration_ns;
    for (size_t i = 0; i < s->ntasks; i++) s->tasks[i].next_release = t0 + s->tasks[i].offset_ns;

    for (;;) {
        if (stop && atomic_load_explicit(stop, memory_order_relaxed)) break;

        /* earliest pending release */
        uint64_t next = UINT64_MAX;
        for (size_t i = 0; i < s->ntasks; i++)
            if (s->tasks[i].next_release < next) next = s->tasks[i].next_release;
        if (next >= end) break;

        kc_sleep_until(next, s->spin_ns);

        /* dispatch every ready task, highest priority first */
        for (;;) {
            uint64_t now = kc_now_ns();
            kt_task *best = NULL;
            for (size_t i = 0; i < s->ntasks; i++) {
                kt_task *t = &s->tasks[i];
                if (t->next_release <= now && (!best || t->priority < best->priority)) best = t;
            }
            if (!best) break;

            uint64_t release = best->next_release;
            uint64_t start = kc_now_ns();
            best->fn(best->ctx, release);
            uint64_t done = kc_now_ns();

            kh_record(&best->jitter_ns, start - release);
            kh_record(&best->exec_ns, done - start);
            best->runs++;

            uint64_t deadline = release + best->period_ns;
            if (done > deadline) best->deadline_misses++;
            best->next_release = deadline;
            /* fell more than a full period behind: skip, don't burst-catch-up */
            while (best->next_release + best->period_ns <= done) {
                best->next_release += best->period_ns;
                best->skipped_periods++;
            }
        }
    }
}
