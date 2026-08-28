# Session summary — gyroflow_stock_i2c

## Objective

Debug the heap corruption that crashes the M5Stack Core2 GCSV gyro logger at clip end (branch `gyroflow_stock_i2c`). The logger samples the MPU6886 at 1 kHz over I2C (14-byte reads at 400 kHz) and writes GCSV rows to an SD card. The goal is to identify the corruptor and fix it so full clips record cleanly.

## Important Details

- **TWO standalone isolation tests are now DEFINITIVELY CLEAN:**
  1. **SD stress test** (commit `98449e3`): 14 MB written to `/sdtest.bin` via 4096-byte `File::write` chunks + full read-back verify = **0 mismatches, no heap corruption, no crash**. The SD card / SPI / FatFs write path is **NOT** the corruptor. Measured **~311 KB/s** raw SD write throughput.
  2. **IMU 1 kHz sampling test** (commit `3b919dd`, branch `gyroflow_stock_i2c`): sampled the MPU6886 at 1 kHz (14-byte I2C burst at 400 kHz, exactly as `GyroLogWriter::poll()` does) into a 64 KB PSRAM ring for 30 s = **29995 samples, 0 I2C failures, heap integrity OK the entire time, NO SD at all**. So the 1 kHz I2C read path / sampling loop is **NOT** the corruptor either.
- **CONCLUSION: neither the SD write path nor the 1 kHz I2C sampling path corrupts the heap ON ITS OWN.** The heap corruption therefore appears **ONLY under the CONCURRENT combination** of 1 kHz I2C IMU reads + SD writes running together. This is a **real concurrency/timing bug**, not a simple buffer overflow in either subsystem alone.
- **Leading hypotheses for the concurrency bug:**
  - (a) the I2C (internal bus) and SPI (SD) drivers sharing/contending on something — DMA channels, a shared buffer, or interrupt priority inversion;
  - (b) the SD write happening in the same `loop()` iteration as the I2C read causing a specific ordering issue;
  - (c) the 1 kHz sampling filling the ring AND the drain-to-SD happening concurrently with I2C reads in a way that the two separate isolation tests (which run each subsystem alone) never exercise.
- The corruptor is still not directly identified, but it is now **constrained to the concurrent I2C+SD interaction**.
- Practical constraint: 30-min clips = ~54 MB, so whole-clip PSRAM buffering is out; the ring+drain design is required — the fix must make I2C and SD not corrupt each other.

## Work State

### Completed

- Standalone SD stress test implemented and run: 14 MB write + full read-back verify, 0 mismatches, heap intact, no crash (commit `98449e3`). SD card / SPI / FatFs write path ruled out.
- Standalone 1 kHz IMU sampling test implemented and run: 30 s at 1 kHz into the 64 KB PSRAM ring, 29995 samples, 0 I2C failures, heap integrity OK, no SD (commit `3b919dd`). The 1 kHz I2C read path / sampling loop ruled out.
- **Both isolation tests clean → the corruptor is constrained to the CONCURRENT I2C+SD interaction** (a real concurrency/timing bug).
- Raw SD write throughput characterized (~311 KB/s at 4096-byte chunks).
- GyroLogWriter converted to the 64 KB PSRAM ring version (committed); SD writes rate-limited to 20 Hz in ≤4 KB chunks.
- Prior diagnostic builds (mount-only, open/write isolation, raw-FatFs GCSV, 1 kHz no-SD isolation) landed on the branch.

### Active

- Bisecting the concurrency: confirming the real full `GyroLogWriter` (1 kHz sampling + drain-to-SD, both I2C and SD active) reproduces the heap corruption with the 1 Hz `[DIAG]` heap-integrity check in `poll()` enabled.

### Blocked

- None.

## Next Move

Run the **actual full `GyroLogWriter`** (1 kHz sampling + drain-to-SD, the real code path) with the `[DIAG]` heap-integrity check in `poll()` enabled, and confirm it reproduces the corruption — then bisect the concurrency.

1. **Confirm** the real logger (both I2C and SD active) reproduces the heap corruption with the 1 Hz heap check.
2. If it does, the fix direction is to **DECOUPLE the I2C sampling from the SD writes in time** — e.g., do the I2C read and the SD drain in strictly separate, non-overlapping phases, or move the SD drain to a dedicated low-priority task / use a double-buffer so the I2C read never happens while an SD write is in flight. The cleanest robust fix is likely to run the SD writes from a **separate FreeRTOS task** (or a dedicated I2C task) so the two buses never contend in the same critical window.

## Relevant Files

- `src/main/main-m5stack-core2.cpp` — both diag tests: `sdStressTest()` (gated by `#define SD_STRESS_TEST`, now 0) and `imuSampleTest()` (gated by `#define IMU_SAMPLE_TEST`, now 1), both called at the top of `setup()` after `M5.begin()`; `imuSampleTest()` does `M5.In_I2C.writeRegister8(0x68,0x19,0x00,400000)` to set 1 kHz ODR, then a loop reading 14 bytes from reg 0x3B at 400 kHz, formatting GCSV-style rows into a `ps_malloc`'d 64 KB ring, checking `heap_caps_check_integrity_all(true)` every second. Also the `SMPLRT_DIV=0` 1 kHz bump.
- `src/GyroLog/GyroLogWriter.cpp` / `src/GyroLog/GyroLogWriter.h` — the 64 KB PSRAM ring `poll()`/`drainRing()`/`readImu()` (committed); the corruptor is constrained to the concurrent I2C+SD interaction.
- `platformio.ini` — build config for the M5Stack Core2 target.
