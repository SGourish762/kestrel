/* bench_decode - cost of the per-frame processing stage, and of raw signal
 * extraction for Intel vs Motorola layouts. Single thread, no I/O. */
#include <stdio.h>
#include <stdlib.h>

#include "kestrel/clock.h"
#include "kestrel/proc.h"
#include "kestrel/sim.h"

#define NFRAMES (1u << 16)

int main(void) {
    /* pre-generate a realistic frame mix with the simulator */
    kcan_frame *frames = malloc(NFRAMES * sizeof *frames);
    ksim sim;
    ksim_init(&sim, 3, 0.0);
    unsigned n = 0;
    for (uint64_t t = 5000000; n < NFRAMES; t += 5000000)
        for (int m = 0; m < KMSG_COUNT && n < NFRAMES; m++)
            if (t % ((uint64_t)KDBC_VEHICLE[m].period_ms * 1000000ull) == 0 && ksim_frame(&sim, m, t, &frames[n]))
                n++;

    /* raw extraction */
    volatile double sink = 0;
    const kcan_signal *intel = &KDBC_VEHICLE[KMSG_WHEEL].sigs[1];
    const kcan_signal *moto = &KDBC_VEHICLE[KMSG_IMU].sigs[1];
    const unsigned R = 200;
    uint64_t t0 = kc_now_ns();
    for (unsigned r = 0; r < R; r++)
        for (unsigned i = 0; i < NFRAMES; i++) sink += kcan_decode(frames[i].data, intel);
    uint64_t t1 = kc_now_ns();
    for (unsigned r = 0; r < R; r++)
        for (unsigned i = 0; i < NFRAMES; i++) sink += kcan_decode(frames[i].data, moto);
    uint64_t t2 = kc_now_ns();
    double per = (double)R * NFRAMES;
    printf("signal decode   Intel %.2f ns | Motorola %.2f ns per signal\n", (double)(t1 - t0) / per,
           (double)(t2 - t1) / per);

    /* full processing stage: lookup + E2E + decode all signals + fusion */
    kproc *p = malloc(sizeof *p);
    double best = 1e18;
    for (int rep = 0; rep < 5; rep++) {
        kproc_init(p, false);
        uint64_t a = kc_now_ns();
        for (unsigned r = 0; r < 20; r++) {
            /* reset per-pass so the E2E counters see a fresh, valid sequence */
            for (int m = 0; m < KMSG_COUNT; m++) p->e2e[m].seen = false;
            p->last_imu_t = 0;
            for (unsigned i = 0; i < NFRAMES; i++) kproc_handle(p, &frames[i]);
        }
        double ns = (double)(kc_now_ns() - a) / (20.0 * NFRAMES);
        if (ns < best) best = ns;
    }
    printf("full pipeline   %.1f ns/frame  ->  %.2f M frames/s on one core\n", best, 1e3 / best);
    printf("                (a fully loaded 1 Mbit/s CAN bus carries ~8,000 frames/s)\n");
    printf("                accepted %llu, crc errors %llu\n", (unsigned long long)p->st.accepted,
           (unsigned long long)p->st.crc_errors);
    (void)sink;
    free(p);
    free(frames);
    return 0;
}
