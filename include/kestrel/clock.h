/* clock.h - monotonic time + precise absolute sleeps. */
#ifndef KESTREL_CLOCK_H
#define KESTREL_CLOCK_H

#include <stdint.h>
#include <time.h>

#define KC_NS_PER_SEC 1000000000ull

static inline uint64_t kc_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * KC_NS_PER_SEC + (uint64_t)ts.tv_nsec;
}

/* Hint to the CPU that we are in a spin-wait loop. */
static inline void kc_cpu_relax(void) {
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield" ::: "memory");
#endif
}

/*
 * Sleep until the absolute monotonic time `abs_ns`.
 * Hybrid strategy: block in the kernel until `spin_ns` before the deadline,
 * then busy-wait the remainder. Trading a little CPU for much lower wake-up
 * jitter is the standard trick in soft real-time loops.
 */
void kc_sleep_until(uint64_t abs_ns, uint64_t spin_ns);

#endif
