/* kestrel - real-time vehicle telemetry engine (CLI). */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kestrel/engine.h"

static void usage(void) {
    fprintf(stderr,
            "usage: kestrel [options]\n"
            "  -d, --duration SEC   simulate for SEC seconds (default 5)\n"
            "  -f, --faults RATE    inject faults (drop/corrupt/slip/glitch) at RATE, e.g. 0.01\n"
            "  -s, --seed N         simulator seed (default 42)\n"
            "      --rt             real-time mode: SCHED_FIFO + CPU pinning + mlockall (Linux),\n"
            "                       user-interactive QoS (macOS)\n"
            "      --busy-poll      consumer spins instead of sleeping (lowest latency, burns a core)\n"
            "      --spin US        busy-wait window before each release (default 100)\n"
            "      --record FILE    write simulated bus traffic as a candump log\n"
            "  -r, --replay FILE    replay a candump log (can-utils `candump -l` format)\n"
            "      --fast           replay as fast as possible (throughput test)\n"
            "      --truth          score replay against the simulator's ground truth\n"
            "      --gen FILE       generate --duration seconds of traffic offline and exit\n"
            "      --json           print the report as JSON\n"
            "  -v, --verbose        live status once per second\n");
}

int main(int argc, char **argv) {
    kengine_cfg cfg;
    kengine_default_cfg(&cfg);
    const char *gen = NULL;
    int json = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
#define ARG() (i + 1 < argc ? argv[++i] : (usage(), exit(2), (char *)NULL))
        if (!strcmp(a, "-d") || !strcmp(a, "--duration")) cfg.duration_s = atof(ARG());
        else if (!strcmp(a, "-f") || !strcmp(a, "--faults")) cfg.fault_rate = atof(ARG());
        else if (!strcmp(a, "-s") || !strcmp(a, "--seed")) cfg.seed = strtoull(ARG(), NULL, 10);
        else if (!strcmp(a, "--rt")) cfg.realtime = true;
        else if (!strcmp(a, "--busy-poll")) cfg.busy_poll = true;
        else if (!strcmp(a, "--spin")) cfg.spin_ns = strtoull(ARG(), NULL, 10) * 1000ull;
        else if (!strcmp(a, "--record")) cfg.record_path = ARG();
        else if (!strcmp(a, "-r") || !strcmp(a, "--replay")) cfg.replay_path = ARG();
        else if (!strcmp(a, "--fast")) cfg.replay_fast = true;
        else if (!strcmp(a, "--truth")) cfg.score_truth = true;
        else if (!strcmp(a, "--gen")) gen = ARG();
        else if (!strcmp(a, "--json")) json = 1;
        else if (!strcmp(a, "-v") || !strcmp(a, "--verbose")) cfg.verbose = true;
        else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(); return 0; }
        else { fprintf(stderr, "unknown option: %s\n", a); usage(); return 2; }
#undef ARG
    }
    if (cfg.duration_s <= 0) { fprintf(stderr, "duration must be > 0\n"); return 2; }

    if (gen) {
        if (kengine_generate(gen, cfg.duration_s, cfg.fault_rate, cfg.seed) != 0) { perror(gen); return 1; }
        fprintf(stderr, "wrote %.1f s of traffic to %s\n", cfg.duration_s, gen);
        return 0;
    }

    kengine_report *rep = malloc(sizeof *rep);
    if (!rep) return 1;
    if (kengine_run(&cfg, rep) != 0) { fprintf(stderr, "engine failed to start\n"); free(rep); return 1; }
    if (cfg.realtime && !rep->rt_ok)
        fprintf(stderr, "note: real-time priority not granted (needs root/CAP_SYS_NICE on Linux); ran best-effort\n");
    if (json) kengine_print_json(rep, stdout);
    else kengine_print(rep, stdout);
    free(rep);
    return 0;
}
