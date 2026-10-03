#include "kestrel/ring.h"

#include <stdlib.h>
#include <string.h>

static size_t next_pow2(size_t v) {
    size_t p = 1;
    while (p < v) p <<= 1;
    return p;
}

int kr_ring_init(kr_ring *r, size_t min_capacity, size_t elem_size) {
    if (!r || min_capacity == 0 || elem_size == 0) return -1;
    size_t cap = next_pow2(min_capacity);
    void *buf = NULL;
    if (posix_memalign(&buf, KR_CACHELINE, cap * elem_size) != 0) return -1;
    atomic_init(&r->head, 0);
    atomic_init(&r->tail, 0);
    r->cached_tail = 0;
    r->cached_head = 0;
    r->cap = cap;
    r->mask = cap - 1;
    r->elem_size = elem_size;
    r->buf = buf;
    return 0;
}

void kr_ring_free(kr_ring *r) {
    if (!r) return;
    free(r->buf);
    r->buf = NULL;
}

bool kr_ring_push(kr_ring *r, const void *elem) {
    size_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    if (h - r->cached_tail == r->cap) {
        r->cached_tail = atomic_load_explicit(&r->tail, memory_order_acquire);
        if (h - r->cached_tail == r->cap) return false;
    }
    memcpy(r->buf + (h & r->mask) * r->elem_size, elem, r->elem_size);
    atomic_store_explicit(&r->head, h + 1, memory_order_release);
    return true;
}

bool kr_ring_pop(kr_ring *r, void *out) {
    size_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
    if (t == r->cached_head) {
        r->cached_head = atomic_load_explicit(&r->head, memory_order_acquire);
        if (t == r->cached_head) return false;
    }
    memcpy(out, r->buf + (t & r->mask) * r->elem_size, r->elem_size);
    atomic_store_explicit(&r->tail, t + 1, memory_order_release);
    return true;
}

size_t kr_ring_pop_batch(kr_ring *r, void *out, size_t max) {
    size_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
    size_t avail = r->cached_head - t;
    if (avail == 0) {
        r->cached_head = atomic_load_explicit(&r->head, memory_order_acquire);
        avail = r->cached_head - t;
        if (avail == 0) return 0;
    }
    size_t n = avail < max ? avail : max;
    unsigned char *dst = out;
    for (size_t i = 0; i < n; i++)
        memcpy(dst + i * r->elem_size, r->buf + ((t + i) & r->mask) * r->elem_size, r->elem_size);
    /* One release store publishes the whole batch's slots back to the producer. */
    atomic_store_explicit(&r->tail, t + n, memory_order_release);
    return n;
}

size_t kr_ring_size(kr_ring *r) {
    size_t h = atomic_load_explicit(&r->head, memory_order_acquire);
    size_t t = atomic_load_explicit(&r->tail, memory_order_acquire);
    return h - t;
}
