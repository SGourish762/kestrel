#include "kestrel/proc.h"

#include <math.h>
#include <string.h>

#include "kestrel/sim.h"

void kproc_init(kproc *p, bool score_truth) {
    memset(p, 0, sizeof *p);
    /* q: accel noise incl. IMU bias; r_vel: wheel avg noise; r_pos: GPS; gate: 4 sigma */
    kf_init(&p->kf, 0.3, 0.12, 1.0, 16.0);
    p->score_truth = score_truth;
}

static void score(kproc *p, uint64_t t_ns, double wheel_v) {
    double tp, tv;
    ksim_truth((double)t_ns / 1e9, &tp, &tv, NULL);
    double ev = p->kf.x[1] - tv, ep = p->kf.x[0] - tp, er = wheel_v - tv;
    p->se_vel += ev * ev;
    p->se_pos += ep * ep;
    p->se_vel_raw += er * er;
    p->n_scored++;
}

bool kproc_handle(kproc *p, const kcan_frame *f) {
    p->st.frames++;
    int mi = kdbc_find(f->id, f->flags);
    if (mi < 0) { p->st.unknown_id++; return false; }
    const kdbc_message *m = &KDBC_VEHICLE[mi];
    if (f->dlc != m->dlc) { p->st.bad_dlc++; return false; }

    if (m->e2e) {
        uint8_t lost = 0;
        switch (ke2e_check(f, &p->e2e[mi], &lost)) {
        case KE2E_OK: break;
        case KE2E_OK_LOST: p->st.lost_frames += lost; break;
        case KE2E_ERR_CRC: p->st.crc_errors++; return false;
        case KE2E_ERR_REPEAT: p->st.repeats++; return false;
        default: p->st.bad_dlc++; return false;
        }
    }

    for (uint8_t i = 0; i < m->nsig; i++) {
        const kcan_signal *s = &m->sigs[i];
        p->value[s->slot] = kcan_decode(f->data, s);
        p->value_t[s->slot] = f->t_ns;
    }
    p->st.signals_decoded += m->nsig;
    p->last_seen[mi] = f->t_ns;
    p->seen[mi] = true;
    p->timed_out[mi] = false;
    p->st.accepted++;

    switch (mi) {
    case KMSG_IMU:
        if (p->last_imu_t && f->t_ns > p->last_imu_t)
            kf_predict(&p->kf, p->value[KSIG_IMU_AX], (double)(f->t_ns - p->last_imu_t) / 1e9);
        p->last_imu_t = f->t_ns;
        break;
    case KMSG_WHEEL: {
        double w[4] = { p->value[KSIG_WHEEL_FL], p->value[KSIG_WHEEL_FR],
                        p->value[KSIG_WHEEL_RL], p->value[KSIG_WHEEL_RR] };
        double v = kf_wheel_speed(w) / 3.6;
        kf_update_vel(&p->kf, v);
        if (p->score_truth && p->kf.initialized) score(p, f->t_ns, w[0] / 3.6);
        break;
    }
    case KMSG_GPS:
        kf_update_pos(&p->kf, p->value[KSIG_GPS_POS]);
        break;
    default:
        break;
    }
    return true;
}

void kproc_check_timeouts(kproc *p, uint64_t now_t) {
    for (int i = 0; i < KMSG_COUNT; i++) {
        if (!p->seen[i] || p->timed_out[i]) continue;
        uint64_t limit = 3ull * KDBC_VEHICLE[i].period_ms * 1000000ull;
        if (now_t > p->last_seen[i] + limit) {
            p->timed_out[i] = true;
            p->st.timeouts++;
        }
    }
}

double kproc_rmse_vel(const kproc *p) { return p->n_scored ? sqrt(p->se_vel / (double)p->n_scored) : 0.0; }
double kproc_rmse_pos(const kproc *p) { return p->n_scored ? sqrt(p->se_pos / (double)p->n_scored) : 0.0; }
double kproc_rmse_vel_raw(const kproc *p) { return p->n_scored ? sqrt(p->se_vel_raw / (double)p->n_scored) : 0.0; }
