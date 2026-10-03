#include "kestrel/dbc.h"

/* Wheel speeds: 4 x 12-bit Intel, 0.1 km/h. Byte 6 = counter, byte 7 = CRC. */
static const kcan_signal SIG_WHEEL[] = {
    { "WheelSpeed_FL", "km/h", KSIG_WHEEL_FL, 0, 12, KCAN_INTEL, false, 0.1, 0.0 },
    { "WheelSpeed_FR", "km/h", KSIG_WHEEL_FR, 12, 12, KCAN_INTEL, false, 0.1, 0.0 },
    { "WheelSpeed_RL", "km/h", KSIG_WHEEL_RL, 24, 12, KCAN_INTEL, false, 0.1, 0.0 },
    { "WheelSpeed_RR", "km/h", KSIG_WHEEL_RR, 36, 12, KCAN_INTEL, false, 0.1, 0.0 },
};
/* IMU: 3 x signed 16-bit Motorola (big-endian), as many IMU vendors ship. */
static const kcan_signal SIG_IMU[] = {
    { "LongAccel", "m/s^2", KSIG_IMU_AX, 7, 16, KCAN_MOTOROLA, true, 0.001, 0.0 },
    { "LatAccel", "m/s^2", KSIG_IMU_AY, 23, 16, KCAN_MOTOROLA, true, 0.001, 0.0 },
    { "YawRate", "deg/s", KSIG_IMU_YAW, 39, 16, KCAN_MOTOROLA, true, 0.01, 0.0 },
};
/* Battery: unprotected status frame. */
static const kcan_signal SIG_BATT[] = {
    { "PackVoltage", "V", KSIG_BATT_V, 0, 16, KCAN_INTEL, false, 0.01, 0.0 },
    { "PackCurrent", "A", KSIG_BATT_I, 16, 16, KCAN_INTEL, true, 0.1, 0.0 },
    { "StateOfCharge", "%", KSIG_BATT_SOC, 32, 10, KCAN_INTEL, false, 0.1, 0.0 },
    { "PackTemp", "degC", KSIG_BATT_TEMP, 42, 8, KCAN_INTEL, true, 1.0, 0.0 },
};
/* GPS (1-D along-track for the fusion demo). Sat count shares byte 6 with the counter. */
static const kcan_signal SIG_GPS[] = {
    { "GpsPosition", "m", KSIG_GPS_POS, 0, 32, KCAN_INTEL, true, 0.01, 0.0 },
    { "GpsSpeed", "m/s", KSIG_GPS_SPD, 32, 16, KCAN_INTEL, false, 0.01, 0.0 },
    { "GpsSats", "", KSIG_GPS_SATS, 52, 4, KCAN_INTEL, false, 1.0, 0.0 },
};

#define N(a) (uint8_t)(sizeof(a) / sizeof((a)[0]))

const kdbc_message KDBC_VEHICLE[KMSG_COUNT] = {
    [KMSG_WHEEL] = { KDBC_ID_WHEEL, "WHEEL_SPEEDS", 10, 8, 1, SIG_WHEEL, N(SIG_WHEEL) },
    [KMSG_IMU] = { KDBC_ID_IMU, "IMU", 5, 8, 1, SIG_IMU, N(SIG_IMU) },
    [KMSG_BATT] = { KDBC_ID_BATT, "BATTERY", 100, 8, 0, SIG_BATT, N(SIG_BATT) },
    [KMSG_GPS] = { KDBC_ID_GPS, "GPS", 100, 8, 1, SIG_GPS, N(SIG_GPS) },
};

/* 11-bit ID space is only 2048 entries: a const direct-mapped table beats
 * hashing. Stored as index+1 so the zero-initialized default means "unknown". */
static const uint8_t ID_TABLE[2048] = {
    [KDBC_ID_WHEEL] = KMSG_WHEEL + 1,
    [KDBC_ID_IMU] = KMSG_IMU + 1,
    [KDBC_ID_BATT] = KMSG_BATT + 1,
    [KDBC_ID_GPS] = KMSG_GPS + 1,
};

int kdbc_find(uint32_t id, uint8_t flags) {
    if ((flags & KCAN_FLAG_EXT) || id > 0x7FF) return -1;
    return (int)ID_TABLE[id] - 1;
}
