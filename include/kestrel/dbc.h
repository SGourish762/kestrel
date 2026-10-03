/*
 * dbc.h - the vehicle message database (what a .dbc file describes).
 * Defines the messages kestrel understands plus an O(1) ID lookup.
 */
#ifndef KESTREL_DBC_H
#define KESTREL_DBC_H

#include <stddef.h>
#include <stdint.h>

#include "kestrel/can.h"

enum {
    KSIG_WHEEL_FL, KSIG_WHEEL_FR, KSIG_WHEEL_RL, KSIG_WHEEL_RR,
    KSIG_IMU_AX, KSIG_IMU_AY, KSIG_IMU_YAW,
    KSIG_BATT_V, KSIG_BATT_I, KSIG_BATT_SOC, KSIG_BATT_TEMP,
    KSIG_GPS_POS, KSIG_GPS_SPD, KSIG_GPS_SATS,
    KSIG_COUNT
};

enum { KMSG_WHEEL, KMSG_IMU, KMSG_BATT, KMSG_GPS, KMSG_COUNT };

#define KDBC_ID_WHEEL 0x0D0u
#define KDBC_ID_IMU 0x0E0u
#define KDBC_ID_BATT 0x132u
#define KDBC_ID_GPS 0x3E9u

typedef struct {
    uint32_t id;
    const char *name;
    uint32_t period_ms;
    uint8_t dlc;
    uint8_t e2e; /* frame carries counter + CRC */
    const kcan_signal *sigs;
    uint8_t nsig;
} kdbc_message;

extern const kdbc_message KDBC_VEHICLE[KMSG_COUNT];

/* Returns message index (0..KMSG_COUNT-1) or -1. O(1) for 11-bit IDs. */
int kdbc_find(uint32_t id, uint8_t flags);

#endif
