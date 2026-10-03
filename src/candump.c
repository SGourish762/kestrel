#include "kestrel/candump.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int kcd_parse(const char *s, kcan_frame *f) {
    memset(f, 0, sizeof *f);
    while (isspace((unsigned char)*s)) s++;
    if (*s != '(') return -1;
    s++;
    uint64_t sec = 0, frac = 0;
    int ndig = 0;
    if (!isdigit((unsigned char)*s)) return -1;
    while (isdigit((unsigned char)*s)) { sec = sec * 10 + (uint64_t)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (isdigit((unsigned char)*s)) {
            if (ndig < 9) { frac = frac * 10 + (uint64_t)(*s - '0'); ndig++; }
            s++;
        }
    }
    while (ndig < 9) { frac *= 10; ndig++; }
    if (*s != ')') return -1;
    s++;
    f->t_ns = sec * 1000000000ull + frac;

    /* interface name */
    while (*s == ' ') s++;
    if (!*s || isspace((unsigned char)*s)) return -1;
    while (*s && !isspace((unsigned char)*s)) s++;
    while (*s == ' ') s++;

    /* ID */
    const char *id0 = s;
    uint32_t id = 0;
    int h;
    while ((h = hexval(*s)) >= 0) { id = (id << 4) | (uint32_t)h; s++; }
    size_t idlen = (size_t)(s - id0);
    if (*s != '#') return -1;
    s++;
    if (idlen == 3) {
        if (id > 0x7FF) return -1;
    } else if (idlen == 8) {
        if (id > 0x1FFFFFFF) return -1;
        f->flags |= KCAN_FLAG_EXT;
    } else {
        return -1;
    }
    if (*s == '#') return 1; /* CAN FD */
    if (*s == 'R' || *s == 'r') return 1; /* remote frame */
    f->id = id;

    /* data */
    int n = 0;
    while (hexval(s[0]) >= 0) {
        if (hexval(s[1]) < 0 || n >= 8) return -1;
        f->data[n++] = (uint8_t)((hexval(s[0]) << 4) | hexval(s[1]));
        s += 2;
        if (*s == '.') s++; /* candump allows '.' separators */
    }
    while (*s == ' ' || *s == '\r' || *s == '\n') s++;
    if (*s) return -1;
    f->dlc = (uint8_t)n;
    return 0;
}

int kcd_format(char *buf, size_t n, const kcan_frame *f, const char *ifname) {
    char data[17] = {0};
    for (int i = 0; i < f->dlc && i < 8; i++) snprintf(data + 2 * i, 3, "%02X", f->data[i]);
    int w;
    if (f->flags & KCAN_FLAG_EXT)
        w = snprintf(buf, n, "(%010llu.%06llu) %s %08X#%s", (unsigned long long)(f->t_ns / 1000000000ull),
                     (unsigned long long)((f->t_ns % 1000000000ull) / 1000), ifname, (unsigned)f->id, data);
    else
        w = snprintf(buf, n, "(%010llu.%06llu) %s %03X#%s", (unsigned long long)(f->t_ns / 1000000000ull),
                     (unsigned long long)((f->t_ns % 1000000000ull) / 1000), ifname, (unsigned)f->id, data);
    if (w < 0 || (size_t)w >= n) return -1;
    return w;
}
