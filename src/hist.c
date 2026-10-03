#include "kestrel/hist.h"

#include <string.h>

void kh_init(kh_hist *h) {
    memset(h, 0, sizeof *h);
    h->min = UINT64_MAX;
}

unsigned kh_bucket_of(uint64_t v) {
    if (v < 2 * KH_SUB) return (unsigned)v;
    unsigned msb = 63u - (unsigned)__builtin_clzll(v);
    unsigned shift = msb - KH_SUB_BITS;
    unsigned mant = (unsigned)(v >> shift); /* in [KH_SUB, 2*KH_SUB) */
    return (shift + 1) * KH_SUB + (mant - KH_SUB);
}

uint64_t kh_bucket_low(unsigned idx) {
    if (idx < 2 * KH_SUB) return idx;
    unsigned shift = idx / KH_SUB - 1;
    uint64_t mant = idx % KH_SUB + KH_SUB;
    return mant << shift;
}

uint64_t kh_bucket_high(unsigned idx) {
    if (idx < 2 * KH_SUB) return idx;
    unsigned shift = idx / KH_SUB - 1;
    uint64_t mant = idx % KH_SUB + KH_SUB;
    return ((mant + 1) << shift) - 1;
}

void kh_record(kh_hist *h, uint64_t v) {
    h->counts[kh_bucket_of(v)]++;
    h->total++;
    h->sum += (long double)v;
    if (v < h->min) h->min = v;
    if (v > h->max) h->max = v;
}

void kh_merge(kh_hist *dst, const kh_hist *src) {
    for (unsigned i = 0; i < KH_BUCKETS; i++) dst->counts[i] += src->counts[i];
    dst->total += src->total;
    dst->sum += src->sum;
    if (src->min < dst->min) dst->min = src->min;
    if (src->max > dst->max) dst->max = src->max;
}

uint64_t kh_percentile(const kh_hist *h, double p) {
    if (h->total == 0) return 0;
    if (p <= 0.0) return h->min;
    if (p >= 100.0) return h->max;
    uint64_t target = (uint64_t)((p / 100.0) * (double)h->total + 0.999999);
    if (target == 0) target = 1;
    uint64_t seen = 0;
    for (unsigned i = 0; i < KH_BUCKETS; i++) {
        seen += h->counts[i];
        if (seen >= target) {
            uint64_t lo = kh_bucket_low(i), hi = kh_bucket_high(i);
            uint64_t mid = lo + (hi - lo) / 2;
            if (mid < h->min) mid = h->min;
            if (mid > h->max) mid = h->max;
            return mid;
        }
    }
    return h->max;
}

double kh_mean(const kh_hist *h) {
    return h->total ? (double)(h->sum / (long double)h->total) : 0.0;
}
