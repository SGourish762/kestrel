#include "kestrel/clock.h"

#include <errno.h>

void kc_sleep_until(uint64_t abs_ns, uint64_t spin_ns) {
    for (;;) {
        uint64_t now = kc_now_ns();
        if (now >= abs_ns) return;
        uint64_t rem = abs_ns - now;
        if (rem <= spin_ns) {
            while (kc_now_ns() < abs_ns) kc_cpu_relax();
            return;
        }
        uint64_t wake = abs_ns - spin_ns;
#if defined(__linux__)
        /* Absolute sleep: immune to drift from time spent before the call. */
        struct timespec ts = { (time_t)(wake / KC_NS_PER_SEC), (long)(wake % KC_NS_PER_SEC) };
        int rc;
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } while (rc == EINTR);
#else
        /* macOS has no clock_nanosleep; fall back to a relative sleep and loop. */
        uint64_t d = wake - now;
        struct timespec ts = { (time_t)(d / KC_NS_PER_SEC), (long)(d % KC_NS_PER_SEC) };
        nanosleep(&ts, NULL);
#endif
    }
}
