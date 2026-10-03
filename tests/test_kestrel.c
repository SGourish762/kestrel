#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "kestrel/can.h"
#include "kestrel/candump.h"
#include "kestrel/clock.h"
#include "kestrel/dbc.h"
#include "kestrel/e2e.h"
#include "kestrel/engine.h"
#include "kestrel/fusion.h"
#include "kestrel/hist.h"
#include "kestrel/proc.h"
#include "kestrel/ring.h"
#include "kestrel/sched.h"
#include "kestrel/sim.h"
#include "test.h"

#if defined(__has_feature)
#if __has_feature(thread_sanitizer) || __has_feature(address_sanitizer)
#define SLOW_BUILD 1
#endif
#endif
#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
#define SLOW_BUILD 1
#endif
#ifndef SLOW_BUILD
#define SLOW_BUILD 0
#endif

/* ---------------- ring ---------------- */

static void test_ring_basic(void) {
    kr_ring r;
    CHECK_EQ(kr_ring_init(&r, 5, sizeof(int)), 0);
    CHECK_EQ(r.cap, 8); /* rounded to power of two */
    int v;
    CHECK(!kr_ring_pop(&r, &v));
    for (int i = 0; i < 8; i++) CHECK(kr_ring_push(&r, &i));
    int x = 99;
    CHECK(!kr_ring_push(&r, &x)); /* full */
    CHECK_EQ(kr_ring_size(&r), 8);
    for (int i = 0; i < 8; i++) {
        CHECK(kr_ring_pop(&r, &v));
        CHECK_EQ(v, i);
    }
    CHECK(!kr_ring_pop(&r, &v));
    /* wrap-around many times */
    for (int i = 0; i < 1000; i++) {
        CHECK(kr_ring_push(&r, &i));
        CHECK(kr_ring_pop(&r, &v));
        if (v != i) { CHECK_EQ(v, i); break; }
    }
    kr_ring_free(&r);
}

static void test_ring_batch(void) {
    kr_ring r;
    kr_ring_init(&r, 16, sizeof(uint32_t));
    for (uint32_t i = 0; i < 10; i++) kr_ring_push(&r, &i);
    uint32_t out[16];
    CHECK_EQ(kr_ring_pop_batch(&r, out, 4), 4);
    CHECK_EQ(out[3], 3);
    CHECK_EQ(kr_ring_pop_batch(&r, out, 16), 6);
    CHECK_EQ(out[5], 9);
    CHECK_EQ(kr_ring_pop_batch(&r, out, 16), 0);
    kr_ring_free(&r);
}

typedef struct {
    kr_ring *r;
    uint64_t n;
    uint64_t errors;
} stress_arg;

static void *stress_producer(void *p) {
    stress_arg *a = p;
    for (uint64_t i = 0; i < a->n; i++)
        while (!kr_ring_push(a->r, &i)) kc_cpu_relax();
    return NULL;
}

static void test_ring_threaded(void) {
    kr_ring r;
    kr_ring_init(&r, 1024, sizeof(uint64_t));
    stress_arg a = { &r, SLOW_BUILD ? 200000 : 5000000, 0 };
    pthread_t th;
    pthread_create(&th, NULL, stress_producer, &a);
    uint64_t expect = 0, v, buf[32];
    while (expect < a.n) {
        if ((expect & 1) == 0) {
            if (kr_ring_pop(&r, &v)) { if (v != expect) a.errors++; expect++; }
        } else {
            size_t k = kr_ring_pop_batch(&r, buf, 32);
            for (size_t i = 0; i < k; i++, expect++) if (buf[i] != expect) a.errors++;
        }
    }
    pthread_join(th, NULL);
    CHECK_EQ(a.errors, 0); /* every element arrives exactly once, in order */
    CHECK_EQ(kr_ring_size(&r), 0);
    kr_ring_free(&r);
}

/* ---------------- CAN bit packing ---------------- */

static void test_can_intel(void) {
    uint8_t d[8] = { 0xAB, 0xCD, 0, 0, 0, 0, 0, 0 };
    CHECK_EQ(kcan_extract_raw(d, 0, 16, KCAN_INTEL), 0xCDAB);
    CHECK_EQ(kcan_extract_raw(d, 4, 12, KCAN_INTEL), 0xCDA); /* (d0>>4)|(d1<<4) */
    CHECK_EQ(kcan_extract_raw(d, 0, 4, KCAN_INTEL), 0xB);
}

static void test_can_motorola(void) {
    uint8_t d[8] = { 0xAB, 0xCD, 0xEF, 0, 0, 0, 0, 0 };
    /* start bit 7 = MSB of byte 0, 16 bits -> big-endian bytes 0..1 */
    CHECK_EQ(kcan_extract_raw(d, 7, 16, KCAN_MOTOROLA), 0xABCD);
    /* start bit 3, 12 bits: low nibble of byte 0 then all of byte 1 */
    CHECK_EQ(kcan_extract_raw(d, 3, 12, KCAN_MOTOROLA), 0xBCD);
    /* start bit 15 (byte 1 MSB), 8 bits */
    CHECK_EQ(kcan_extract_raw(d, 15, 8, KCAN_MOTOROLA), 0xCD);
    /* bit-by-bit reference walk of the sawtooth for a 1-bit signal */
    CHECK_EQ(kcan_extract_raw(d, 0, 1, KCAN_MOTOROLA), 1);  /* LSB of 0xAB */
    CHECK_EQ(kcan_extract_raw(d, 6, 1, KCAN_MOTOROLA), 0);  /* bit6 of 0xAB = 0 */
}

static void test_can_signed_scaled(void) {
    kcan_signal s = { "x", "", 0, 7, 16, KCAN_MOTOROLA, true, 0.001, 0.0 };
    uint8_t d[8] = { 0 };
    CHECK_EQ(kcan_encode(d, &s, -1.5), 0);
    CHECK_EQ(d[0], 0xFA); /* -1500 = 0xFA24 */
    CHECK_EQ(d[1], 0x24);
    CHECK_NEAR(kcan_decode(d, &s), -1.5, 1e-9);
    CHECK_EQ(kcan_encode(d, &s, 40.0), -1); /* 40000 > int16 max */
    CHECK_NEAR(kcan_decode(d, &s), -1.5, 1e-9); /* untouched on failure */

    kcan_signal t = { "t", "", 0, 42, 8, KCAN_INTEL, true, 1.0, -40.0 };
    memset(d, 0xFF, 8);
    CHECK_EQ(kcan_encode(d, &t, 25.0), 0);
    CHECK_NEAR(kcan_decode(d, &t), 25.0, 1e-9);
    CHECK_EQ(d[0], 0xFF); /* neighbours preserved */
    CHECK_EQ(d[7], 0xFF);
}

/* Reference implementation: one bit at a time, straight from the DBC spec. */
static uint64_t ref_extract(const uint8_t d[8], int start, int len, int order) {
    uint64_t raw = 0;
    if (order == KCAN_INTEL) {
        for (int i = len - 1; i >= 0; i--) {
            int pos = start + i;
            raw = (raw << 1) | ((d[pos / 8] >> (pos % 8)) & 1u);
        }
    } else {
        int pos = start;
        for (int i = 0; i < len; i++) {
            raw = (raw << 1) | ((d[pos / 8] >> (pos % 8)) & 1u);
            pos = (pos % 8 == 0) ? pos + 15 : pos - 1;
        }
    }
    return raw;
}

static void test_can_fuzz_vs_reference(void) {
    uint64_t rng = 12345;
    int mismatches = 0, roundtrip_fail = 0, tested = 0;
    for (int it = 0; it < 200000; it++) {
        kcan_signal s = { "f", "", 0, (uint8_t)(ksim_uniform(&rng) * 64), (uint8_t)(1 + ksim_uniform(&rng) * 64),
                          (uint8_t)(ksim_uniform(&rng) < 0.5 ? KCAN_INTEL : KCAN_MOTOROLA), false, 1, 0 };
        if (!kcan_signal_valid(&s)) continue;
        tested++;
        uint8_t d[8];
        for (int i = 0; i < 8; i++) d[i] = (uint8_t)(ksim_uniform(&rng) * 256);
        if (kcan_extract_raw(d, s.start_bit, s.length, (kcan_order)s.order) !=
            ref_extract(d, s.start_bit, s.length, s.order))
            mismatches++;
        /* insert then extract returns the value and leaves other bits alone */
        uint8_t e[8];
        memcpy(e, d, 8);
        uint64_t val = ((uint64_t)(ksim_uniform(&rng) * 4294967296.0) << 32) | (uint64_t)(ksim_uniform(&rng) * 4294967296.0);
        uint64_t m = s.length == 64 ? ~0ull : ((1ull << s.length) - 1);
        kcan_insert_raw(e, s.start_bit, s.length, (kcan_order)s.order, val);
        if (kcan_extract_raw(e, s.start_bit, s.length, (kcan_order)s.order) != (val & m)) roundtrip_fail++;
        uint64_t smask = kcan_signal_mask(&s), dl = 0, el = 0;
        for (int i = 7; i >= 0; i--) { dl = (dl << 8) | d[i]; el = (el << 8) | e[i]; }
        if ((dl & ~smask) != (el & ~smask)) roundtrip_fail++;
    }
    CHECK(tested > 50000);
    CHECK_EQ(mismatches, 0);
    CHECK_EQ(roundtrip_fail, 0);
}

/* ---------------- CRC / E2E ---------------- */

static void test_crc8_vector(void) {
    const uint8_t msg[] = "123456789";
    CHECK_EQ(ke2e_crc8_j1850(msg, 9), 0x4B); /* published check value for CRC-8/SAE-J1850 */
}

static void test_e2e(void) {
    kcan_frame f = { 0 };
    f.id = 0x123;
    f.dlc = 8;
    ke2e_state st = { 0 };
    uint8_t lost;
    for (uint8_t c = 0; c < 40; c++) { /* wraps 15 -> 0 twice */
        ke2e_protect(&f, c);
        CHECK_EQ(ke2e_check(&f, &st, &lost), KE2E_OK);
    }
    CHECK_EQ(ke2e_check(&f, &st, &lost), KE2E_ERR_REPEAT);
    ke2e_protect(&f, 40 + 3); /* skipped 3 */
    CHECK_EQ(ke2e_check(&f, &st, &lost), KE2E_OK_LOST);
    CHECK_EQ(lost, 3);
    ke2e_protect(&f, 44);
    f.data[2] ^= 0x10; /* single bit flip */
    CHECK_EQ(ke2e_check(&f, &st, &lost), KE2E_ERR_CRC);
    f.data[2] ^= 0x10;
    f.id = 0x124; /* right payload, wrong ID: CRC covers the ID */
    CHECK_EQ(ke2e_check(&f, &st, &lost), KE2E_ERR_CRC);
}

static void test_crc_detects_all_single_and_double_bit_errors(void) {
    kcan_frame f = { 0 };
    f.id = 0x0E0;
    f.dlc = 8;
    for (int i = 0; i < 6; i++) f.data[i] = (uint8_t)(i * 37 + 11);
    ke2e_protect(&f, 5);
    int undetected = 0;
    for (int a = 0; a < 64; a++) {
        for (int b = a; b < 64; b++) {
            kcan_frame g = f;
            ke2e_state st = { 4, true };
            g.data[a / 8] ^= (uint8_t)(1u << (a % 8));
            if (b != a) g.data[b / 8] ^= (uint8_t)(1u << (b % 8));
            uint8_t lost;
            if (ke2e_check(&g, &st, &lost) != KE2E_ERR_CRC) undetected++;
        }
    }
    CHECK_EQ(undetected, 0); /* HD=4 for short messages: all 1- and 2-bit errors caught */
}

/* ---------------- DBC ---------------- */

static void test_dbc_layout(void) {
    for (int m = 0; m < KMSG_COUNT; m++) {
        const kdbc_message *msg = &KDBC_VEHICLE[m];
        CHECK_EQ(kdbc_find(msg->id, 0), m);
        uint64_t used = 0;
        if (msg->e2e) used = (0xFFull << 56) | (0x0Full << 48); /* CRC byte + counter nibble */
        for (int i = 0; i < msg->nsig; i++) {
            const kcan_signal *s = &msg->sigs[i];
            CHECK(kcan_signal_valid(s));
            uint64_t mk = kcan_signal_mask(s);
            if (used & mk) fprintf(stderr, "    overlap: %s.%s\n", msg->name, s->name);
            CHECK_EQ(used & mk, 0); /* no two signals share a bit */
            used |= mk;
        }
    }
    CHECK_EQ(kdbc_find(0x7FF, 0), -1);
    CHECK_EQ(kdbc_find(KDBC_ID_IMU, KCAN_FLAG_EXT), -1);
}

/* ---------------- candump ---------------- */

static void test_candump(void) {
    kcan_frame f;
    CHECK_EQ(kcd_parse("(1436509052.249713) vcan0 0D0#0102030405060708\n", &f), 0);
    CHECK_EQ(f.id, 0x0D0);
    CHECK_EQ(f.dlc, 8);
    CHECK_EQ(f.data[7], 0x08);
    CHECK_EQ(f.t_ns, 1436509052249713000ull);
    char buf[128];
    CHECK(kcd_format(buf, sizeof buf, &f, "vcan0") > 0);
    CHECK(strcmp(buf, "(1436509052.249713) vcan0 0D0#0102030405060708") == 0);

    CHECK_EQ(kcd_parse("(1.5) can1 1ABCDEF0#DEAD", &f), 0);
    CHECK(f.flags & KCAN_FLAG_EXT);
    CHECK_EQ(f.id, 0x1ABCDEF0);
    CHECK_EQ(f.dlc, 2);
    CHECK_EQ(kcd_parse("(1.0) can0 123#", &f), 0);
    CHECK_EQ(f.dlc, 0);
    CHECK_EQ(kcd_parse("(1.0) can0 123#R", &f), 1);
    CHECK_EQ(kcd_parse("(1.0) can0 123##1AA", &f), 1);
    CHECK_EQ(kcd_parse("garbage", &f), -1);
    CHECK_EQ(kcd_parse("(1.0) can0 800#00", &f), -1);           /* >11-bit with 3 digits */
    CHECK_EQ(kcd_parse("(1.0) can0 123#001122334455667788", &f), -1); /* 9 bytes */
    CHECK_EQ(kcd_parse("(1.0) can0 123#0", &f), -1);            /* odd nibble */
}

/* ---------------- histogram ---------------- */

static void test_hist(void) {
    for (uint64_t v = 0; v < 100000; v += 7) {
        unsigned b = kh_bucket_of(v);
        if (!(kh_bucket_low(b) <= v && v <= kh_bucket_high(b))) { CHECK(0); break; }
    }
    CHECK(kh_bucket_of(UINT64_MAX) < KH_BUCKETS);
    kh_hist *h = malloc(sizeof *h);
    kh_init(h);
    for (uint64_t v = 1; v <= 100000; v++) kh_record(h, v);
    CHECK_EQ(h->min, 1);
    CHECK_EQ(h->max, 100000);
    CHECK_NEAR(kh_percentile(h, 50), 50000, 50000 * 0.03);
    CHECK_NEAR(kh_percentile(h, 99), 99000, 99000 * 0.03);
    CHECK_NEAR(kh_mean(h), 50000.5, 1e-6);
    free(h);
}

/* ---------------- fusion ---------------- */

static void test_wheel_speed_robust(void) {
    double w[4] = { 20.0, 20.2, 27.0, 19.9 }; /* one wheel slipping */
    CHECK_NEAR(kf_wheel_speed(w), 20.1, 1e-9);
}

static void test_kalman_gate(void) {
    kf1d kf;
    kf_init(&kf, 0.3, 0.12, 1.0, 16.0);
    for (int i = 0; i < 200; i++) {
        kf_predict(&kf, 0.0, 0.01);
        kf_update_vel(&kf, 10.0);
        if (i % 10 == 0) kf_update_pos(&kf, 10.0 * i * 0.01);
    }
    CHECK_NEAR(kf.x[1], 10.0, 0.05);
    CHECK(!kf_update_pos(&kf, kf.x[0] + 50.0)); /* 50 m GPS jump rejected */
    CHECK(!kf_update_vel(&kf, 30.0));           /* impossible speed rejected */
    CHECK(kf_update_vel(&kf, 10.05));
}

/* Offline end-to-end: simulator -> processing, no threads, exact bookkeeping. */
static void run_offline(double seconds, double faults, kproc *p, ksim *s) {
    ksim_init(s, 7, faults);
    kproc_init(p, true);
    for (uint64_t t = 5000000; t <= (uint64_t)(seconds * 1e9); t += 5000000) {
        for (int m = 0; m < KMSG_COUNT; m++) {
            if (t % ((uint64_t)KDBC_VEHICLE[m].period_ms * 1000000ull)) continue;
            kcan_frame f;
            if (ksim_frame(s, m, t, &f)) kproc_handle(p, &f);
        }
        kproc_check_timeouts(p, t);
    }
}

static void test_pipeline_clean(void) {
    kproc *p = malloc(sizeof *p);
    ksim s;
    run_offline(60, 0.0, p, &s);
    CHECK_EQ(p->st.crc_errors, 0);
    CHECK_EQ(p->st.lost_frames, 0);
    CHECK_EQ(p->st.timeouts, 0);
    CHECK_EQ(p->st.accepted, s.frames);
    CHECK(kproc_rmse_vel(p) < 0.10);
    CHECK(kproc_rmse_vel(p) < kproc_rmse_vel_raw(p));
    CHECK(kproc_rmse_pos(p) < 1.0);
    free(p);
}

static void test_pipeline_faults(void) {
    kproc *p = malloc(sizeof *p);
    ksim s;
    run_offline(120, 0.01, p, &s);
    CHECK(s.inj_corrupt > 100);
    CHECK_EQ(p->st.crc_errors, s.inj_corrupt); /* every corrupted frame caught */
    /* every gap is seen by the alive counter: drops on protected messages plus
     * CRC-rejected frames (which also never advance the receiver's counter) */
    uint64_t expect_lost = s.inj_drop[KMSG_WHEEL] + s.inj_drop[KMSG_IMU] + s.inj_drop[KMSG_GPS] + s.inj_corrupt;
    CHECK_NEAR((double)p->st.lost_frames, (double)expect_lost, 6.0); /* tail-of-run gaps are invisible */
    CHECK(p->kf.rejected_pos >= s.inj_gps_glitch * 9 / 10);          /* GPS jumps gated out */
    CHECK(kproc_rmse_vel(p) < 0.15);
    CHECK(kproc_rmse_pos(p) < 1.5);
    free(p);
}

static void test_timeout_detection(void) {
    kproc *p = malloc(sizeof *p);
    ksim s;
    ksim_init(&s, 1, 0.0);
    kproc_init(p, false);
    for (uint64_t t = 5000000; t <= 2000000000ull; t += 5000000) {
        for (int m = 0; m < KMSG_COUNT; m++) {
            if (t % ((uint64_t)KDBC_VEHICLE[m].period_ms * 1000000ull)) continue;
            if (m == KMSG_GPS && t > 1000000000ull) continue; /* GPS goes silent at 1 s */
            kcan_frame f;
            if (ksim_frame(&s, m, t, &f)) kproc_handle(p, &f);
        }
        kproc_check_timeouts(p, t);
    }
    CHECK_EQ(p->st.timeouts, 1); /* edge-triggered: reported once */
    CHECK(p->timed_out[KMSG_GPS]);
    CHECK(!p->timed_out[KMSG_IMU]);
    free(p);
}

/* ---------------- scheduler ---------------- */

static void count_task(void *ctx, uint64_t rel) {
    (void)rel;
    (*(int *)ctx)++;
}

static void test_sched(void) {
    CHECK_NEAR(kt_rm_bound(1), 1.0, 1e-12);
    CHECK_NEAR(kt_rm_bound(3), 0.7797631, 1e-6);
    int a = 0, b = 0;
    kt_task t[2];
    kt_task_init(&t[0], "fast", count_task, &a, 2000000, 0, 1000, 0);
    kt_task_init(&t[1], "slow", count_task, &b, 10000000, 500000, 1000, 1);
    CHECK_NEAR(kt_utilization(t, 2), 0.0006, 1e-9);
    kt_sched s = { t, 2, 50000 };
    kt_sched_run(&s, 200000000ull, NULL); /* 200 ms */
    /* Every period is either run or explicitly counted as skipped, so this
     * holds exactly even on an overloaded CI VM (macOS runners oversleep). */
    uint64_t fast_periods = t[0].runs + t[0].skipped_periods;
    uint64_t slow_periods = t[1].runs + t[1].skipped_periods;
    CHECK(fast_periods >= 98 && fast_periods <= 101);
    CHECK(slow_periods >= 19 && slow_periods <= 21);
    CHECK_EQ(a, t[0].runs);
    CHECK_EQ(b, t[1].runs);
    CHECK(a >= 50); /* sanity: the loop actually ran most periods */
}

/* ---------------- threaded engine smoke test ---------------- */

static void test_engine_sim(void) {
    kengine_cfg c;
    kengine_default_cfg(&c);
    c.duration_s = 1.0;
    c.fault_rate = 0.01;
    kengine_report *r = malloc(sizeof *r);
    CHECK_EQ(kengine_run(&c, r), 0);
    CHECK(r->proc.st.frames > 250);
    CHECK_EQ(r->proc.st.frames, r->pushed);
    CHECK_EQ(r->proc.st.crc_errors, r->sim.inj_corrupt);
    free(r);
}

int main(void) {
    printf("kestrel tests%s\n", SLOW_BUILD ? " (sanitizer build)" : "");
    RUN(test_ring_basic);
    RUN(test_ring_batch);
    RUN(test_ring_threaded);
    RUN(test_can_intel);
    RUN(test_can_motorola);
    RUN(test_can_signed_scaled);
    RUN(test_can_fuzz_vs_reference);
    RUN(test_crc8_vector);
    RUN(test_e2e);
    RUN(test_crc_detects_all_single_and_double_bit_errors);
    RUN(test_dbc_layout);
    RUN(test_candump);
    RUN(test_hist);
    RUN(test_wheel_speed_robust);
    RUN(test_kalman_gate);
    RUN(test_pipeline_clean);
    RUN(test_pipeline_faults);
    RUN(test_timeout_detection);
    RUN(test_sched);
    RUN(test_engine_sim);
    printf("%d checks, %d failures\n", t_checks, t_failures);
    return t_failures ? 1 : 0;
}
