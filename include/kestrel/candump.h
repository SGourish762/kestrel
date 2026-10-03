/*
 * candump.h - read/write the Linux can-utils `candump -l` log format:
 *   (1436509052.249713) vcan0 0D0#0102030405060708
 * so kestrel can replay real captures and produce logs other tools read.
 */
#ifndef KESTREL_CANDUMP_H
#define KESTREL_CANDUMP_H

#include <stddef.h>

#include "kestrel/can.h"

/* 0 = parsed, 1 = valid but skipped (remote/FD/error frame), -1 = malformed. */
int kcd_parse(const char *line, kcan_frame *f);
/* Returns chars written (excluding NUL) or -1 if buf too small. */
int kcd_format(char *buf, size_t n, const kcan_frame *f, const char *ifname);

#endif
