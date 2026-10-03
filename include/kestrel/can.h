/*
 * can.h - CAN frames and DBC-style signal packing/unpacking.
 *
 * Bit numbering follows the Vector DBC convention used across the industry:
 *  - Intel (little-endian, "@1"): start_bit is the signal's LSB; bits ascend.
 *  - Motorola (big-endian, "@0"): start_bit is the signal's MSB, numbered
 *    byte*8 + bit_in_byte (bit 7 = MSB of the byte), walking the classic
 *    "sawtooth" pattern toward higher bytes.
 *
 * Both orders are implemented branch-light by loading the 8 payload bytes as
 * one 64-bit word (LE or BE) and doing a single shift + mask.
 */
#ifndef KESTREL_CAN_H
#define KESTREL_CAN_H

#include <stdbool.h>
#include <stdint.h>

#define KCAN_FLAG_EXT 0x01u /* 29-bit extended identifier */
#define KCAN_MAX_DLC 8

typedef struct {
    uint64_t t_ns;   /* bus/source timestamp (sim time or log time) */
    uint64_t rx_ns;  /* monotonic receive timestamp, used for latency */
    uint32_t id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t data[KCAN_MAX_DLC];
} kcan_frame;

typedef enum { KCAN_INTEL = 0, KCAN_MOTOROLA = 1 } kcan_order;

typedef struct {
    const char *name;
    const char *unit;
    uint16_t slot;      /* index into the signal store */
    uint8_t start_bit;
    uint8_t length;     /* 1..64 */
    uint8_t order;      /* kcan_order */
    bool is_signed;
    double scale;
    double offset;
} kcan_signal;

/* Validate that a signal definition fits inside an 8-byte payload. */
bool kcan_signal_valid(const kcan_signal *s);
/* Bitmask (over the LE-loaded 64-bit payload) of bits the signal occupies. */
uint64_t kcan_signal_mask(const kcan_signal *s);

uint64_t kcan_extract_raw(const uint8_t data[8], uint8_t start, uint8_t len, kcan_order o);
void kcan_insert_raw(uint8_t data[8], uint8_t start, uint8_t len, kcan_order o, uint64_t raw);

/* Raw -> physical: phys = raw * scale + offset (raw sign-extended if signed). */
double kcan_decode(const uint8_t data[8], const kcan_signal *s);
/* Physical -> raw, rounded. Returns -1 (payload untouched) if out of range. */
int kcan_encode(uint8_t data[8], const kcan_signal *s, double phys);

#endif
