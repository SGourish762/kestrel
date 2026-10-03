#include "kestrel/can.h"

#include <math.h>
#include <string.h>

/* Payload <-> 64-bit word. memcpy + byte swap compiles to a single load
 * (plus one REV/BSWAP instruction) on every mainstream compiler. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define TO_LE(x) __builtin_bswap64(x)
#define TO_BE(x) (x)
#else
#define TO_LE(x) (x)
#define TO_BE(x) __builtin_bswap64(x)
#endif
static inline uint64_t load_le64(const uint8_t d[8]) {
    uint64_t v;
    memcpy(&v, d, 8);
    return TO_LE(v);
}
static inline uint64_t load_be64(const uint8_t d[8]) {
    uint64_t v;
    memcpy(&v, d, 8);
    return TO_BE(v);
}
static inline void store_le64(uint8_t d[8], uint64_t v) {
    v = TO_LE(v);
    memcpy(d, &v, 8);
}
static inline void store_be64(uint8_t d[8], uint64_t v) {
    v = TO_BE(v);
    memcpy(d, &v, 8);
}
static inline uint64_t low_mask(uint8_t len) {
    return len >= 64 ? ~0ull : ((1ull << len) - 1);
}
/* Motorola start bit (MSB, sawtooth numbering) -> linear index from the
 * most-significant bit of the big-endian-loaded payload word. */
static inline unsigned moto_lin(uint8_t start) {
    return (unsigned)(start / 8) * 8u + (7u - (start % 8u));
}

bool kcan_signal_valid(const kcan_signal *s) {
    if (!s || s->length == 0 || s->length > 64 || s->start_bit > 63) return false;
    if (s->scale == 0.0) return false;
    if (s->order == KCAN_INTEL) return (unsigned)s->start_bit + s->length <= 64;
    if (s->order == KCAN_MOTOROLA) return moto_lin(s->start_bit) + s->length <= 64;
    return false;
}

uint64_t kcan_signal_mask(const kcan_signal *s) {
    uint8_t d[8] = {0};
    kcan_insert_raw(d, s->start_bit, s->length, (kcan_order)s->order, ~0ull);
    return load_le64(d);
}

uint64_t kcan_extract_raw(const uint8_t data[8], uint8_t start, uint8_t len, kcan_order o) {
    if (o == KCAN_INTEL) return (load_le64(data) >> start) & low_mask(len);
    unsigned shift = 64u - moto_lin(start) - len;
    return (load_be64(data) >> shift) & low_mask(len);
}

void kcan_insert_raw(uint8_t data[8], uint8_t start, uint8_t len, kcan_order o, uint64_t raw) {
    raw &= low_mask(len);
    if (o == KCAN_INTEL) {
        uint64_t m = low_mask(len) << start;
        uint64_t v = load_le64(data);
        store_le64(data, (v & ~m) | (raw << start));
    } else {
        unsigned shift = 64u - moto_lin(start) - len;
        uint64_t m = low_mask(len) << shift;
        uint64_t v = load_be64(data);
        store_be64(data, (v & ~m) | (raw << shift));
    }
}

double kcan_decode(const uint8_t data[8], const kcan_signal *s) {
    uint64_t raw = kcan_extract_raw(data, s->start_bit, s->length, (kcan_order)s->order);
    if (s->is_signed && s->length < 64 && (raw >> (s->length - 1)) & 1u)
        raw |= ~low_mask(s->length); /* sign-extend */
    double v = s->is_signed ? (double)(int64_t)raw : (double)raw;
    return v * s->scale + s->offset;
}

int kcan_encode(uint8_t data[8], const kcan_signal *s, double phys) {
    double r = (phys - s->offset) / s->scale;
    if (!isfinite(r)) return -1;
    if (s->length < 64) {
        if (s->is_signed) {
            double lo = -ldexp(1.0, s->length - 1), hi = ldexp(1.0, s->length - 1) - 1.0;
            r = nearbyint(r);
            if (r < lo || r > hi) return -1;
        } else {
            r = nearbyint(r);
            if (r < 0.0 || r > ldexp(1.0, s->length) - 1.0) return -1;
        }
    } else {
        r = nearbyint(r);
    }
    uint64_t raw = s->is_signed ? (uint64_t)(int64_t)r : (uint64_t)r;
    kcan_insert_raw(data, s->start_bit, s->length, (kcan_order)s->order, raw);
    return 0;
}
