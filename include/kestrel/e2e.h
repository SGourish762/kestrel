/*
 * e2e.h - End-to-end message protection (modeled on AUTOSAR E2E Profile 1).
 *
 * Protected frames carry:
 *   data[dlc-1]            CRC-8/SAE-J1850 over {id_lo, id_hi, data[0..dlc-2]}
 *   data[dlc-2] & 0x0F     4-bit rolling alive counter
 * Folding the CAN ID into the CRC catches frames delivered under the wrong ID;
 * the counter catches lost, repeated, and stale frames.
 */
#ifndef KESTREL_E2E_H
#define KESTREL_E2E_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kestrel/can.h"

uint8_t ke2e_crc8(const uint8_t *p, size_t n, uint8_t crc); /* SAE J1850, poly 0x1D */
uint8_t ke2e_crc8_j1850(const uint8_t *p, size_t n);        /* init 0xFF, xorout 0xFF */

typedef enum {
    KE2E_OK = 0,
    KE2E_OK_LOST,  /* accepted, but counter jumped: frames were lost */
    KE2E_ERR_CRC,
    KE2E_ERR_REPEAT,
    KE2E_ERR_DLC,
} ke2e_status;

typedef struct {
    uint8_t last_counter;
    bool seen;
} ke2e_state;

int ke2e_protect(kcan_frame *f, uint8_t counter);
ke2e_status ke2e_check(const kcan_frame *f, ke2e_state *st, uint8_t *lost);

#endif
