/*
 * ring.h - bounded, lock-free single-producer/single-consumer ring buffer.
 *
 * Design notes:
 *  - head/tail are monotonically increasing indices; slot = index & mask.
 *    Capacity is rounded up to a power of two so the modulo is a single AND.
 *  - Producer owns `head`, consumer owns `tail`. Each lives on its own cache
 *    line (128 B on Apple Silicon, 64 B elsewhere) to avoid false sharing.
 *  - Each side keeps a *cached* copy of the other side's index and only
 *    re-reads the shared atomic when the cache says full/empty. In steady
 *    state this removes nearly all cross-core cache-line traffic.
 *  - Memory ordering: the element is written, then `head` is published with
 *    release; the consumer reads `head` with acquire before reading the
 *    element (and symmetrically for `tail`). No locks, no CAS loops.
 */
#ifndef KESTREL_RING_H
#define KESTREL_RING_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>

#if defined(__APPLE__) && defined(__aarch64__)
#define KR_CACHELINE 128
#else
#define KR_CACHELINE 64
#endif

typedef struct kr_ring {
    /* producer cache line */
    _Alignas(KR_CACHELINE) _Atomic size_t head;
    size_t cached_tail;
    /* consumer cache line */
    _Alignas(KR_CACHELINE) _Atomic size_t tail;
    size_t cached_head;
    /* read-only after init */
    _Alignas(KR_CACHELINE) size_t cap;
    size_t mask;
    size_t elem_size;
    unsigned char *buf;
} kr_ring;

/* Returns 0 on success. `min_capacity` is rounded up to a power of two. */
int kr_ring_init(kr_ring *r, size_t min_capacity, size_t elem_size);
void kr_ring_free(kr_ring *r);

/* Producer side. Returns false if the ring is full. */
bool kr_ring_push(kr_ring *r, const void *elem);
/* Consumer side. Returns false if the ring is empty. */
bool kr_ring_pop(kr_ring *r, void *out);
/* Consumer side: pop up to `max` elements into `out`. Returns count popped. */
size_t kr_ring_pop_batch(kr_ring *r, void *out, size_t max);

/* Approximate occupancy (exact only when both sides are quiescent). */
size_t kr_ring_size(kr_ring *r);

#endif
