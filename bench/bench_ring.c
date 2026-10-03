/* bench_ring - lock-free SPSC ring vs. a mutex+condvar queue (the textbook
 * baseline), moving 32-byte CAN frames between two threads. */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kestrel/can.h"
#include "kestrel/clock.h"
#include "kestrel/ring.h"

#define N_ITEMS 20000000ull
#define CAP 4096

/* ---- baseline: bounded queue guarded by a mutex + two condvars ---- */
typedef struct {
    pthread_mutex_t mu;
    pthread_cond_t not_full, not_empty;
    kcan_frame *buf;
    size_t head, tail, count, cap;
} mq;

static void mq_init(mq *q, size_t cap) {
    pthread_mutex_init(&q->mu, NULL);
    pthread_cond_init(&q->not_full, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    q->buf = malloc(cap * sizeof *q->buf);
    q->head = q->tail = q->count = 0;
    q->cap = cap;
}
static void mq_push(mq *q, const kcan_frame *f) {
    pthread_mutex_lock(&q->mu);
    while (q->count == q->cap) pthread_cond_wait(&q->not_full, &q->mu);
    q->buf[q->head] = *f;
    q->head = (q->head + 1) % q->cap;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mu);
}
static void mq_pop(mq *q, kcan_frame *f) {
    pthread_mutex_lock(&q->mu);
    while (q->count == 0) pthread_cond_wait(&q->not_empty, &q->mu);
    *f = q->buf[q->tail];
    q->tail = (q->tail + 1) % q->cap;
    q->count--;
    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->mu);
}

static mq g_mq;
static kr_ring g_ring;

static void *mq_producer(void *a) {
    (void)a;
    kcan_frame f;
    memset(&f, 0, sizeof f);
    for (uint64_t i = 0; i < N_ITEMS; i++) { f.t_ns = i; mq_push(&g_mq, &f); }
    return NULL;
}
static void *ring_producer(void *a) {
    (void)a;
    kcan_frame f;
    memset(&f, 0, sizeof f);
    for (uint64_t i = 0; i < N_ITEMS; i++) {
        f.t_ns = i;
        while (!kr_ring_push(&g_ring, &f)) kc_cpu_relax();
    }
    return NULL;
}

static double run_mq(void) {
    mq_init(&g_mq, CAP);
    pthread_t th;
    uint64_t t0 = kc_now_ns();
    pthread_create(&th, NULL, mq_producer, NULL);
    kcan_frame f;
    uint64_t bad = 0;
    for (uint64_t i = 0; i < N_ITEMS; i++) { mq_pop(&g_mq, &f); bad += f.t_ns != i; }
    pthread_join(th, NULL);
    double s = (double)(kc_now_ns() - t0) / 1e9;
    if (bad) fprintf(stderr, "mutex queue corrupted %llu items\n", (unsigned long long)bad);
    free(g_mq.buf);
    return (double)N_ITEMS / s;
}

static double run_ring(int batch) {
    kr_ring_init(&g_ring, CAP, sizeof(kcan_frame));
    pthread_t th;
    uint64_t t0 = kc_now_ns();
    pthread_create(&th, NULL, ring_producer, NULL);
    kcan_frame fb[64];
    uint64_t i = 0, bad = 0;
    while (i < N_ITEMS) {
        if (batch) {
            size_t n = kr_ring_pop_batch(&g_ring, fb, 64);
            for (size_t k = 0; k < n; k++, i++) bad += fb[k].t_ns != i;
            if (!n) kc_cpu_relax();
        } else if (kr_ring_pop(&g_ring, fb)) {
            bad += fb[0].t_ns != i;
            i++;
        } else {
            kc_cpu_relax();
        }
    }
    pthread_join(th, NULL);
    double s = (double)(kc_now_ns() - t0) / 1e9;
    if (bad) fprintf(stderr, "ring corrupted %llu items\n", (unsigned long long)bad);
    kr_ring_free(&g_ring);
    return (double)N_ITEMS / s;
}

int main(void) {
    printf("moving %llu x %zu-byte frames between 2 threads (capacity %d)\n",
           (unsigned long long)N_ITEMS, sizeof(kcan_frame), CAP);
    double best_mq = 0, best_r = 0, best_rb = 0;
    for (int rep = 0; rep < 3; rep++) {
        double a = run_mq(), b = run_ring(0), c = run_ring(1);
        if (a > best_mq) best_mq = a;
        if (b > best_r) best_r = b;
        if (c > best_rb) best_rb = c;
    }
    printf("  mutex+condvar queue     %8.2f M frames/s\n", best_mq / 1e6);
    printf("  lock-free SPSC ring     %8.2f M frames/s  (%.1fx)\n", best_r / 1e6, best_r / best_mq);
    printf("  lock-free SPSC, batched %8.2f M frames/s  (%.1fx)\n", best_rb / 1e6, best_rb / best_mq);
    return 0;
}
