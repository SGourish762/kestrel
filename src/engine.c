#include "kestrel/engine.h"

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if defined(__linux__)
#include <sys/mman.h>
#elif defined(__APPLE__)
#include <pthread/qos.h>
#endif

#include "kestrel/candump.h"
#include "kestrel/clock.h"
#include "kestrel/ring.h"

typedef struct engine engine;

typedef struct {
    engine *e;
    int msg;
} task_ctx;

struct engine {
    kr_ring ring; /* first: cache-line aligned */
    const kengine_cfg *cfg;
    kengine_report *rep;
    uint64_t t0;
    atomic_bool producer_done;
    atomic_bool stop;
    FILE *record;
    task_ctx tctx[KMSG_COUNT];
};

void kengine_default_cfg(kengine_cfg *c) {
    memset(c, 0, sizeof *c);
    c->duration_s = 5.0;
    c->fault_rate = 0.0;
    c->seed = 42;
    c->ring_cap = 4096;
    c->spin_ns = 100000; /* 100 us */
}

/* ---- real-time thread setup ------------------------------------------- */

static bool make_realtime(int prio, int cpu) {
#if defined(__linux__)
    bool ok = true;
    struct sched_param sp = { .sched_priority = prio };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) ok = false;
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu >= 0 && ncpu > 1) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu % ncpu, &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof set, &set) != 0) ok = false;
    }
    return ok;
#elif defined(__APPLE__)
    (void)prio;
    (void)cpu;
    return pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0) == 0;
#else
    (void)prio;
    (void)cpu;
    return false;
#endif
}

/* ---- producer: simulator driven by the RM scheduler ------------------- */

static void push_frame(engine *e, kcan_frame *f, bool may_block) {
    f->rx_ns = kc_now_ns();
    while (!kr_ring_push(&e->ring, f)) {
        e->rep->ring_full++;
        if (!may_block) return; /* real-time source: never block, count overflow */
        kc_cpu_relax();
    }
    e->rep->pushed++;
    size_t d = kr_ring_size(&e->ring);
    if (d > e->rep->max_depth) e->rep->max_depth = d;
}

static void sim_task(void *ctx, uint64_t release_ns) {
    task_ctx *tc = ctx;
    engine *e = tc->e;
    kcan_frame f;
    if (!ksim_frame(&e->rep->sim, tc->msg, release_ns - e->t0, &f)) return;
    push_frame(e, &f, false);
    if (e->record) {
        char line[96];
        if (kcd_format(line, sizeof line, &f, "vcan0") > 0) fprintf(e->record, "%s\n", line);
    }
}

static void *producer_sim(void *arg) {
    engine *e = arg;
    kengine_report *r = e->rep;
    if (e->cfg->realtime) r->rt_ok = make_realtime(80, 0);

    /* Rate-monotonic priorities: shorter period = higher priority.
     * Offsets stagger releases so tasks rarely contend for the same instant. */
    static const int prio[KMSG_COUNT] = { [KMSG_IMU] = 0, [KMSG_WHEEL] = 1, [KMSG_GPS] = 2, [KMSG_BATT] = 3 };
    static const uint64_t offs_us[KMSG_COUNT] = { [KMSG_IMU] = 0, [KMSG_WHEEL] = 1000, [KMSG_GPS] = 2000, [KMSG_BATT] = 3000 };
    for (int m = 0; m < KMSG_COUNT; m++) {
        e->tctx[m] = (task_ctx){ e, m };
        kt_task_init(&r->tasks[m], KDBC_VEHICLE[m].name, sim_task, &e->tctx[m],
                     (uint64_t)KDBC_VEHICLE[m].period_ms * 1000000ull, offs_us[m] * 1000ull,
                     20000 /* 20 us budget */, prio[m]);
    }
    r->ntasks = KMSG_COUNT;
    r->utilization = kt_utilization(r->tasks, r->ntasks);
    r->rm_bound = kt_rm_bound(r->ntasks);

    kt_sched s = { r->tasks, r->ntasks, e->cfg->spin_ns };
    e->t0 = kc_now_ns();
    kt_sched_run(&s, (uint64_t)(e->cfg->duration_s * 1e9), &e->stop);
    atomic_store_explicit(&e->producer_done, true, memory_order_release);
    return NULL;
}

/* ---- producer: candump replay ----------------------------------------- */

static void *producer_replay(void *arg) {
    engine *e = arg;
    if (e->cfg->realtime) e->rep->rt_ok = make_realtime(80, 0);
    FILE *fp = fopen(e->cfg->replay_path, "r");
    if (!fp) {
        perror(e->cfg->replay_path);
        atomic_store(&e->producer_done, true);
        return NULL;
    }
    char line[256];
    uint64_t log0 = 0, wall0 = 0;
    bool first = true;
    e->t0 = kc_now_ns();
    while (fgets(line, sizeof line, fp) && !atomic_load_explicit(&e->stop, memory_order_relaxed)) {
        kcan_frame f;
        if (kcd_parse(line, &f) != 0) continue;
        if (!e->cfg->replay_fast) {
            if (first) { log0 = f.t_ns; wall0 = kc_now_ns(); first = false; }
            kc_sleep_until(wall0 + (f.t_ns - log0), e->cfg->spin_ns);
        }
        push_frame(e, &f, true);
    }
    fclose(fp);
    atomic_store_explicit(&e->producer_done, true, memory_order_release);
    return NULL;
}

/* ---- consumer --------------------------------------------------------- */

static void *consumer(void *arg) {
    engine *e = arg;
    kengine_report *r = e->rep;
    if (e->cfg->realtime) make_realtime(70, 1);

    kcan_frame batch[64];
    uint64_t idle = 0, next_status = kc_now_ns() + KC_NS_PER_SEC;
    for (;;) {
        size_t n = kr_ring_pop_batch(&e->ring, batch, 64);
        if (n == 0) {
            if (atomic_load_explicit(&e->producer_done, memory_order_acquire) && kr_ring_size(&e->ring) == 0)
                break;
            /* Adaptive idle: spin briefly for latency, then sleep. Never spin
             * forever: under SCHED_FIFO a busy-looping thread trips Linux RT
             * throttling (sched_rt_runtime_us) and gets frozen for ~50 ms/s. */
            if (++idle < 4000) {
                kc_cpu_relax();
            } else if (e->cfg->busy_poll) {
                sched_yield(); /* dedicated core: lowest latency, 100% CPU */
            } else {
                struct timespec ts = { 0, 20000 }; /* 20 us */
                nanosleep(&ts, NULL);
            }
            continue;
        }
        idle = 0;
        for (size_t i = 0; i < n; i++) {
            uint64_t a = kc_now_ns();
            kproc_handle(&r->proc, &batch[i]);
            uint64_t b = kc_now_ns();
            kh_record(&r->process_ns, b - a);
            kh_record(&r->latency_ns, b - batch[i].rx_ns);
            kproc_check_timeouts(&r->proc, batch[i].t_ns);
        }
        if (e->cfg->verbose && kc_now_ns() >= next_status) {
            next_status += KC_NS_PER_SEC;
            const kf1d *kf = &r->proc.kf;
            double tv = 0, tp = 0;
            uint64_t bt = batch[n - 1].t_ns;
            if (r->proc.score_truth) ksim_truth((double)bt / 1e9, &tp, &tv, NULL);
            fprintf(stderr, "[t=%6.2fs] frames=%-8llu v_est=%6.2f m/s (truth %6.2f)  p_err=%6.2f m  p99 lat=%.1f us\n",
                    (double)bt / 1e9, (unsigned long long)r->proc.st.frames, kf->x[1], tv,
                    r->proc.score_truth ? kf->x[0] - tp : 0.0, (double)kh_percentile(&r->latency_ns, 99) / 1e3);
        }
    }
    return NULL;
}

/* ---- run -------------------------------------------------------------- */

int kengine_run(const kengine_cfg *cfg, kengine_report *r) {
    engine *e = NULL;
    if (posix_memalign((void **)&e, KR_CACHELINE, sizeof *e) != 0) return -1;
    memset(e, 0, sizeof *e);
    memset(r, 0, sizeof *r);
    e->cfg = cfg;
    e->rep = r;
    atomic_init(&e->producer_done, false);
    atomic_init(&e->stop, false);
    kh_init(&r->latency_ns);
    kh_init(&r->process_ns);
    r->sim_mode = cfg->replay_path == NULL;
    kproc_init(&r->proc, r->sim_mode || cfg->score_truth);
    ksim_init(&r->sim, cfg->seed, cfg->fault_rate);

    if (kr_ring_init(&e->ring, cfg->ring_cap ? cfg->ring_cap : 4096, sizeof(kcan_frame)) != 0) {
        free(e);
        return -1;
    }
    if (cfg->record_path && r->sim_mode) {
        e->record = fopen(cfg->record_path, "w");
        if (!e->record) perror(cfg->record_path);
    }
#if defined(__linux__)
    if (cfg->realtime) mlockall(MCL_CURRENT | MCL_FUTURE); /* no page faults mid-loop */
#endif

    uint64_t w0 = kc_now_ns();
    pthread_t prod, cons;
    pthread_create(&cons, NULL, consumer, e);
    pthread_create(&prod, NULL, r->sim_mode ? producer_sim : producer_replay, e);
    pthread_join(prod, NULL);
    pthread_join(cons, NULL);
    r->wall_s = (double)(kc_now_ns() - w0) / 1e9;

    if (e->record) fclose(e->record);
    kr_ring_free(&e->ring);
    free(e);
    return 0;
}

int kengine_generate(const char *path, double seconds, double fault_rate, uint64_t seed) {
    FILE *fp = fopen(path, "w");
    if (!fp) return -1;
    ksim sim;
    ksim_init(&sim, seed, fault_rate);
    uint64_t end = (uint64_t)(seconds * 1e9);
    /* step on the 5 ms base tick (GCD of all periods) */
    for (uint64_t t = 5000000; t <= end; t += 5000000) {
        for (int m = 0; m < KMSG_COUNT; m++) {
            uint64_t per = (uint64_t)KDBC_VEHICLE[m].period_ms * 1000000ull;
            if (t % per) continue;
            kcan_frame f;
            if (!ksim_frame(&sim, m, t, &f)) continue;
            char line[96];
            if (kcd_format(line, sizeof line, &f, "vcan0") > 0) fprintf(fp, "%s\n", line);
        }
    }
    fclose(fp);
    return 0;
}

/* ---- reporting -------------------------------------------------------- */

#define US(x) ((double)(x) / 1e3)

void kengine_print(const kengine_report *r, FILE *fp) {
    const kproc_stats *s = &r->proc.st;
    fprintf(fp, "\n== kestrel report ==============================================\n");
    fprintf(fp, "mode            %s%s\n", r->sim_mode ? "simulation" : "replay",
            r->rt_ok ? " (real-time priority)" : "");
    fprintf(fp, "wall time       %.3f s\n", r->wall_s);
    fprintf(fp, "frames          %llu processed, %.0f frames/s, %llu ring-full, max depth %llu\n",
            (unsigned long long)s->frames, (double)s->frames / (r->wall_s > 0 ? r->wall_s : 1),
            (unsigned long long)r->ring_full, (unsigned long long)r->max_depth);
    fprintf(fp, "latency (rx->fused)  p50 %.2f us | p99 %.2f us | p99.9 %.2f us | max %.2f us\n",
            US(kh_percentile(&r->latency_ns, 50)), US(kh_percentile(&r->latency_ns, 99)),
            US(kh_percentile(&r->latency_ns, 99.9)), US(r->latency_ns.max));
    fprintf(fp, "processing cost      p50 %.0f ns | p99 %.0f ns per frame\n",
            (double)kh_percentile(&r->process_ns, 50), (double)kh_percentile(&r->process_ns, 99));

    if (r->sim_mode) {
        fprintf(fp, "\nscheduler (RM, utilization %.4f vs Liu-Layland bound %.4f -> %s)\n", r->utilization,
                r->rm_bound, r->utilization <= r->rm_bound ? "schedulable" : "NOT guaranteed");
        fprintf(fp, "  %-13s %7s %10s %10s %10s %7s\n", "task", "runs", "jit p50", "jit p99", "jit max", "misses");
        for (size_t i = 0; i < r->ntasks; i++) {
            const kt_task *t = &r->tasks[i];
            fprintf(fp, "  %-13s %7llu %8.1fus %8.1fus %8.1fus %7llu\n", t->name, (unsigned long long)t->runs,
                    US(kh_percentile(&t->jitter_ns, 50)), US(kh_percentile(&t->jitter_ns, 99)),
                    US(t->jitter_ns.max), (unsigned long long)t->deadline_misses);
        }
    }

    fprintf(fp, "\nintegrity\n");
    fprintf(fp, "  CRC errors      %llu detected", (unsigned long long)s->crc_errors);
    if (r->sim_mode) fprintf(fp, " / %llu injected", (unsigned long long)r->sim.inj_corrupt);
    fprintf(fp, "\n  lost frames     %llu detected via alive counter\n", (unsigned long long)s->lost_frames);
    fprintf(fp, "  repeats %llu | unknown IDs %llu | bad DLC %llu | timeouts %llu\n",
            (unsigned long long)s->repeats, (unsigned long long)s->unknown_id,
            (unsigned long long)s->bad_dlc, (unsigned long long)s->timeouts);

    const kf1d *kf = &r->proc.kf;
    fprintf(fp, "\nfusion\n");
    fprintf(fp, "  wheel updates   %u accepted, %u rejected by NIS gate", kf->accepted_vel, kf->rejected_vel);
    if (r->sim_mode) fprintf(fp, " (%llu slips injected)", (unsigned long long)r->sim.inj_slip);
    fprintf(fp, "\n  GPS updates     %u accepted, %u rejected by NIS gate", kf->accepted_pos, kf->rejected_pos);
    if (r->sim_mode) fprintf(fp, " (%llu glitches injected)", (unsigned long long)r->sim.inj_gps_glitch);
    fprintf(fp, "\n");
    if (r->proc.score_truth && r->proc.n_scored) {
        fprintf(fp, "  velocity RMSE   %.3f m/s fused vs %.3f m/s raw wheel sensor (%.1fx better)\n",
                kproc_rmse_vel(&r->proc), kproc_rmse_vel_raw(&r->proc),
                kproc_rmse_vel_raw(&r->proc) / kproc_rmse_vel(&r->proc));
        fprintf(fp, "  position RMSE   %.3f m (GPS noise sigma 1.0 m)\n", kproc_rmse_pos(&r->proc));
    }
    fprintf(fp, "================================================================\n");
}

void kengine_print_json(const kengine_report *r, FILE *fp) {
    const kproc_stats *s = &r->proc.st;
    fprintf(fp, "{\"mode\":\"%s\",\"wall_s\":%.3f,\"frames\":%llu,\"frames_per_s\":%.1f,", r->sim_mode ? "sim" : "replay",
            r->wall_s, (unsigned long long)s->frames, (double)s->frames / (r->wall_s > 0 ? r->wall_s : 1));
    fprintf(fp, "\"latency_us\":{\"p50\":%.3f,\"p99\":%.3f,\"p999\":%.3f,\"max\":%.3f},",
            US(kh_percentile(&r->latency_ns, 50)), US(kh_percentile(&r->latency_ns, 99)),
            US(kh_percentile(&r->latency_ns, 99.9)), US(r->latency_ns.max));
    fprintf(fp, "\"crc_errors\":%llu,\"crc_injected\":%llu,\"lost\":%llu,\"timeouts\":%llu,\"ring_full\":%llu,",
            (unsigned long long)s->crc_errors, (unsigned long long)r->sim.inj_corrupt,
            (unsigned long long)s->lost_frames, (unsigned long long)s->timeouts, (unsigned long long)r->ring_full);
    fprintf(fp, "\"deadline_misses\":%llu,", (unsigned long long)(r->ntasks ? r->tasks[0].deadline_misses + r->tasks[1].deadline_misses + r->tasks[2].deadline_misses + r->tasks[3].deadline_misses : 0));
    fprintf(fp, "\"rmse_vel\":%.4f,\"rmse_vel_raw\":%.4f,\"rmse_pos\":%.4f}\n", kproc_rmse_vel(&r->proc),
            kproc_rmse_vel_raw(&r->proc), kproc_rmse_pos(&r->proc));
}
