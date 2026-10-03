# kestrel

**A real-time vehicle telemetry engine in C11.** It takes in CAN bus traffic, checks every frame with automotive end-to-end protection, decodes signals the way DBC files define them, and fuses IMU, wheel-odometry and GPS data with a gated Kalman filter. A two-thread pipeline joined by a lock-free ring buffer runs all of this, and a rate-monotonic scheduler measures its own jitter.

There are no dependencies beyond libc and pthreads. It builds warning-free under `-Wall -Wextra -Wconversion -Werror` with gcc and clang, and it passes AddressSanitizer, UndefinedBehaviorSanitizer and ThreadSanitizer.

```
 ┌──────────────── RX thread (SCHED_FIFO 80, CPU 0) ───────────────┐
 │ rate-monotonic executive                                        │
 │   IMU 200 Hz ─┐                                                 │
 │   WHEEL 100Hz ├─► simulator / candump replay ─► stamp rx time ──┼──┐
 │   GPS 10 Hz  ─┤     (fault injection)                           │  │
 │   BATT 10 Hz ─┘                                                 │  │
 └─────────────────────────────────────────────────────────────────┘  │
                         lock-free SPSC ring (no locks, no CAS)  ◄───┘
 ┌──────────────── processing thread (SCHED_FIFO 70, CPU 1) ───────┐
 │ O(1) ID lookup ► DLC check ► E2E: CRC-8 + alive counter ►        │
 │ DBC signal decode (Intel/Motorola) ► signal store ►              │
 │ Kalman fusion w/ NIS outlier gate ► freshness/timeout supervision│
 │ ► latency histogram                                              │
 └─────────────────────────────────────────────────────────────────┘
```

## Results

These numbers were measured on a 2-vCPU Linux cloud VM. It was not an RT kernel and there was no core isolation, so dedicated hardware should do better. Run `make bench` and `./build/kestrel --rt` to get your own numbers.

| What | Result |
|---|---|
| Scheduler release jitter, `--rt` | **p50 0.2–0.4 µs, p99 0.5–1.6 µs** across all four tasks, 0 deadline misses in 9,600 releases |
| End-to-end latency (rx → fused), `--busy-poll` | **p50 0.8 µs, p99 10 µs** |
| End-to-end latency, `--rt` with a sleeping consumer | p50 15 µs, p99 33 µs, max 65 µs |
| Processing stage, one core | **46 ns/frame → 21.6 M frames/s**. A saturated 1 Mbit/s CAN bus carries about 8 k frames/s |
| Threaded pipeline, saturated | **10.7 M frames/s** end to end, 0 errors |
| Lock-free ring vs. mutex+condvar queue | **14.0 vs 5.2 M frames/s (2.7×)** |
| Signal decode | 4.0 ns Intel, 6.8 ns Motorola, per signal |
| Corrupted frames caught by CRC | **105 / 105 injected**. All 2,080 possible 1- and 2-bit errors are caught, tested exhaustively |
| GPS glitches rejected by the NIS gate | **9 / 9 injected** |
| Velocity error vs. ground truth | **0.019 m/s fused vs 0.573 m/s raw wheel sensor (≈30× better)** with 1% faults |
| Position error | 0.20 m RMSE while GPS noise is σ = 1.0 m |

The full report is in [`docs/sample_report_rt.txt`](docs/sample_report_rt.txt).

## Quick start

```sh
make            # library, CLI, tests, benchmarks
make test       # 20 test groups, ~2,200 checks
make asan tsan  # sanitizer builds of the test suite
make bench      # ring, decode, pipeline benchmarks

./build/kestrel --duration 10 --faults 0.01 -v          # live sim with fault injection
sudo ./build/kestrel --duration 30 --faults 0.01 --rt   # SCHED_FIFO + pinning + mlockall
./build/kestrel --replay data/sample.candump --fast --truth
./build/kestrel --gen my.candump --duration 60          # write a can-utils log
./build/kestrel --json                                  # machine-readable report
```

On macOS, `--rt` uses the user-interactive QoS class instead, because macOS has no `SCHED_FIFO`.

## What's inside

| Module | File | Notes |
|---|---|---|
| Lock-free SPSC ring | `src/ring.c` | Indices count up forever and map to slots with power-of-two masking. Producer and consumer each get their own cache line (128 B on Apple Silicon, 64 B elsewhere) to stop false sharing. Each side caches the other's index, so it only touches the shared atomic when the ring looks full or empty. Ordering is acquire/release only, with no CAS. There is also a batched pop. |
| CAN signal codec | `src/can.c` | Follows the Vector DBC bit numbering for Intel and Motorola ("sawtooth") layouts. A field is read with one 64-bit load, a byte swap and a shift, with no per-bit loop. Signed values are sign-extended. Encoding range-checks and saturates instead of wrapping. It is **fuzzed against a bit-by-bit reference implementation** of the spec over 100k+ random layouts. |
| E2E protection | `src/e2e.c` | Modeled on AUTOSAR E2E Profile 1. A CRC-8/SAE-J1850 covers the CAN ID and the payload, so a frame sent under the wrong ID fails. A 4-bit alive counter detects lost, repeated and stale frames. The CRC uses a const lookup table, so there is no init step and no race. |
| Message database | `src/dbc.c` | Four messages and 14 signals in mixed byte orders. ID lookup is O(1) through a const direct-mapped 2048-entry table. A test proves that no two signals overlap, including the counter and CRC bits. |
| Sensor fusion | `src/fusion.c` | A 2-state Kalman filter where IMU acceleration drives the prediction, wheel speed and GPS drive the updates, and Q comes from a white-noise-acceleration model. A **normalized-innovation-squared gate** (χ², 4σ) rejects GPS multipath jumps. Wheel speed uses the median of four wheels, so one slipping wheel is ignored. |
| RM scheduler | `src/sched.c` | A fixed-priority cyclic executive with absolute-deadline releases, so timing doesn't drift. Sleep is hybrid: `clock_nanosleep(TIMER_ABSTIME)` first, then a short spin. It records per-task jitter and execution histograms, deadline misses and skipped periods, and checks the Liu & Layland utilization bound. |
| Latency histogram | `src/hist.c` | HdrHistogram-style log-linear buckets with about 3% worst-case error across the full `uint64` range. Fixed 15 KB, and recording is O(1) and allocation-free, so it can run inside the real-time loop. |
| Simulator | `src/sim.c` | Analytic ground truth, so filter error is exact at every timestamp. Sensors have bias, Gaussian noise and quantization. Faults include frame drops, bit flips, wheel slip and GPS jumps, and every injection is counted so detection can be scored. |
| candump I/O | `src/candump.c` | Reads and writes the Linux can-utils `candump -l` format, so you can replay captures from real vehicles or other tools. It handles extended IDs and skips remote and FD frames. |
| Pipeline | `src/engine.c` | Two threads. It has an RT setup (`SCHED_FIFO`, CPU affinity, `mlockall`), adaptive idle and a JSON report. |

## Engineering notes

**A spinning `SCHED_FIFO` thread stalls for 48 ms.** The first real-time version had a consumer that busy-polled forever. Its p99 latency came out at **38 ms**, against 13 µs without RT. The cause is Linux RT throttling (`sched_rt_runtime_us` = 950000), which freezes a real-time task that uses 100% of a CPU for the last 50 ms of every second. The fix is an adaptive idle: spin for about 4,000 iterations, then `nanosleep(20 µs)`. With that change, RT p99 is 33 µs and the maximum is 65 µs. `--busy-poll` still exists for a dedicated core without RT priority, where it reaches 0.8 µs p50.

**Start-up transients.** Jitter maxima of 1.5–2.5 ms always landed on the first release. Starting the executive 20 ms later gives thread start-up and first-touch page faults time to settle before the first deadline.

**CRC-rejected frames count as lost.** A frame that fails the CRC never advances the receiver's alive counter, so the next good frame shows a gap. The fault-injection test checks the exact accounting: `lost = drops on protected messages + CRC rejections`.

**Why not one thread?** Decoding takes about 50 ns, so a single thread would keep up with the bus easily. The split is about isolation. In real ECUs, the RX path runs in interrupt or high-priority context and must never block on application logic. The ring makes that boundary explicit, and `ring_full` counts any frames that overflow instead of stalling the bus side.

## Tests

`make test` runs these groups:

- **Ring**: ordering, full and empty states, wrap-around, batching, and a 5M-element two-thread stress test checking that every element arrives exactly once and in order.
- **Codec**: hand-checked Intel and Motorola vectors, signed and scaled round-trips, range rejection that leaves the payload untouched, and fuzzing against a reference implementation.
- **E2E**: the published CRC-8/SAE-J1850 check value (`0x4B`), counter wrap, repeat and skip handling, the wrong-ID case, and **exhaustive 1- and 2-bit error detection**.
- **DBC**: every signal is valid and no bits overlap.
- **candump**: parse and format round-trip, extended IDs, and malformed-input rejection.
- **Histogram**: bucket bounds over the whole range and percentile accuracy.
- **Fusion**: robust wheel speed, NIS gating, and end-to-end accuracy against ground truth, both clean and with 1% faults.
- **Supervision**: a message that goes silent raises exactly one timeout event.
- **Scheduler**: RM bound math and release counts on real timers.
- **Engine**: a threaded smoke test where every frame pushed is processed and every injected corruption is detected.

CI runs on Ubuntu and macOS with gcc and clang, plus ASan+UBSan and TSan jobs.

## Roadmap

- SocketCAN input (`PF_CAN`/`SOCK_RAW`) for live `vcan0` and USB-CAN adapters
- A parser for real `.dbc` files in place of the compiled-in database
- A 2D constant-turn-rate model (using yaw rate and lateral acceleration) in place of the 1D filter
- Futex or eventfd wake-ups so the consumer gets low latency without spinning

## License

MIT
