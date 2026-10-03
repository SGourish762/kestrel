/*
 * hist.h - fixed-memory log-linear latency histogram (HdrHistogram-style).
 *
 * Values < 64 get exact buckets; above that each power-of-two range is split
 * into 32 linear sub-buckets, giving <= ~3% relative error over the full
 * uint64 range in 15 KB with O(1) record (one clz + shifts, no allocation).
 * Safe to call from a real-time loop.
 */
#ifndef KESTREL_HIST_H
#define KESTREL_HIST_H

#include <stdint.h>

#define KH_SUB_BITS 5
#define KH_SUB (1u << KH_SUB_BITS)
#define KH_BUCKETS ((65 - KH_SUB_BITS) * KH_SUB)

typedef struct {
    uint64_t counts[KH_BUCKETS];
    uint64_t total;
    uint64_t min, max;
    long double sum;
} kh_hist;

void kh_init(kh_hist *h);
void kh_record(kh_hist *h, uint64_t v);
void kh_merge(kh_hist *dst, const kh_hist *src);
/* p in [0,100]. Returns a value within one bucket of the true percentile. */
uint64_t kh_percentile(const kh_hist *h, double p);
double kh_mean(const kh_hist *h);

/* exposed for tests */
unsigned kh_bucket_of(uint64_t v);
uint64_t kh_bucket_low(unsigned idx);
uint64_t kh_bucket_high(unsigned idx);

#endif
