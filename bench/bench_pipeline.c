/* bench_pipeline - end-to-end threaded throughput: producer thread pushes
 * pre-generated frames as fast as the ring accepts them; the consumer runs
 * the full processing stage (E2E + decode + fusion) on every frame. */
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>

#include "kestrel/clock.h"
#include "kestrel/hist.h"
#include "kestrel/proc.h"
#include "kestrel/ring.h"
#include "kestrel/sim.h"

#define NGEN (1u << 20) /* ~1M distinct frames = ~55 min of bus traffic */
#define PASSES 8

static kcan_frame *frames;
static kr_ring ring;
static atomic_bool done;

static void *producer(void *a) {
    (void)a;
    for (int p = 0; p < PASSES; p++)
        for (unsigned i = 0; i < NGEN; i++) {
            kcan_frame f = frames[i];
            f.rx_ns = kc_now_ns();
            while (!kr_ring_push(&ring, &f)) kc_cpu_relax();
        }
    atomic_store_explicit(&done, true, memory_order_release);
    return NULL;
}

int main(void) {
    frames = malloc(NGEN * sizeof *frames);
    ksim sim;
    ksim_init(&sim, 11, 0.0);
    unsigned n = 0;
    for (uint64_t t = 5000000; n < NGEN; t += 5000000)
        for (int m = 0; m < KMSG_COUNT && n < NGEN; m++)
            if (t % ((uint64_t)KDBC_VEHICLE[m].period_ms * 1000000ull) == 0 && ksim_frame(&sim, m, t, &frames[n]))
                n++;

    kproc *p = malloc(sizeof *p);
    kproc_init(p, false);
    kh_hist *lat = malloc(sizeof *lat);
    kh_init(lat);
    kr_ring_init(&ring, 4096, sizeof(kcan_frame));

    pthread_t th;
    uint64_t t0 = kc_now_ns();
    pthread_create(&th, NULL, producer, NULL);
    kcan_frame batch[64];
    uint64_t got = 0, pass_frames = 0;
    for (;;) {
        size_t k = kr_ring_pop_batch(&ring, batch, 64);
        if (!k) {
            if (atomic_load_explicit(&done, memory_order_acquire) && kr_ring_size(&ring) == 0) break;
            kc_cpu_relax();
            continue;
        }
        for (size_t i = 0; i < k; i++) {
            if (pass_frames++ == NGEN) { /* replaying the log again: reset sequence state */
                pass_frames = 1;
                for (int m = 0; m < KMSG_COUNT; m++) p->e2e[m].seen = false;
                p->last_imu_t = 0;
            }
            kproc_handle(p, &batch[i]);
            kh_record(lat, kc_now_ns() - batch[i].rx_ns);
        }
        got += k;
    }
    pthread_join(th, NULL);
    double s = (double)(kc_now_ns() - t0) / 1e9;
    printf("pipeline: %llu frames in %.2f s = %.2f M frames/s (2 threads, saturated)\n",
           (unsigned long long)got, s, (double)got / s / 1e6);
    printf("          errors: crc %llu, repeats %llu, lost %llu (expect 0)\n",
           (unsigned long long)p->st.crc_errors, (unsigned long long)p->st.repeats,
           (unsigned long long)p->st.lost_frames);
    printf("          queueing latency under saturation: p50 %.1f us, p99 %.1f us\n",
           (double)kh_percentile(lat, 50) / 1e3, (double)kh_percentile(lat, 99) / 1e3);
    return 0;
}
