# Session summary — gyroflow_stock_i2c

## Objective

Debug the heap corruption that crashes the M5Stack Core2 GCSV gyro logger at clip end (branch `gyroflow_stock_i2c`). The logger samples the MPU6886 at 1 kHz over I2C (14-byte reads at 400 kHz) and writes GCSV rows to an SD card. The goal is to identify the corruptor and fix it so full clips record cleanly.

## Important Details

- **Standalone SD stress test is DEFINITIVELY CLEAN** (commit `98449e3`, branch `gyroflow_stock_i2c`): 14 MB written to `/sdtest.bin` via 4096-byte `File::write` chunks + full read-back verify = **0 mismatches, no heap corruption, no crash**. So the SD card, the SPI bus, and the FatFs write path are **NOT** the corruptor.
- **Raw SD write throughput measured: ~311 KB/s** for 4096-byte chunks.
- Since the SD write path is clean on its own, the heap corruption is **NOT a property of writing to the SD card**. It only appears when the 1 kHz IMU sampling (M5.In_I2C 14-byte reads at 400 kHz) runs **concurrently** with the SD writes.
- Leading hypothesis: the corruption is a **timing/concurrency interaction** between the 1 kHz I2C IMU reads and the SD writes — OR the **I2C read path itself** (the 14-byte read at 400 kHz) is the corruptor, with the SD write merely being the other heavy operation present. The clean standalone SD test rules out the SD/FatFs/sd_diskio write path.
- The corruptor is still unidentified; the SD write path is now ruled out.

## Work State

### Completed

- Standalone SD stress test implemented and run: 14 MB write + full read-back verify, 0 mismatches, heap intact, no crash (commit `98449e3`).
- SD card / SPI / FatFs write path ruled out as the corruptor.
- Raw SD write throughput characterized (~311 KB/s at 4096-byte chunks).
- GyroLogWriter converted to the 64 KB PSRAM ring version (committed); SD writes rate-limited to 20 Hz in ≤4 KB chunks.
- Prior diagnostic builds (mount-only, open/write isolation, raw-FatFs GCSV, 1 kHz no-SD isolation) landed on the branch.

### Active

- Determining whether the corruptor is the 1 kHz I2C read path itself, or the concurrent combination of 1 kHz I2C reads + SD writes.

### Blocked

- None.

## Next Move

Test the **1 kHz IMU sampling ALONE** (no SD writes at all): sample the gyro at 1 kHz into the PSRAM ring for ~30–60 s with **no file writing**, then check heap integrity.

- If that **corrupts the heap** → the I2C read path (or the 1 kHz sampling loop) is the corruptor, not the SD.
- If it's **clean** → the corruption is specifically the **concurrent combination** of 1 kHz I2C reads + SD writes (a real concurrency/timing bug).

## Relevant Files

- `src/main/main-m5stack-core2.cpp` — `sdStressTest()`, gated by `#define SD_STRESS_TEST 1`, called at the top of `setup()` after `M5.begin()` (the standalone SD stress test; now proven clean).
- `src/GyroLog/GyroLogWriter.cpp` / `src/GyroLog/GyroLogWriter.h` — the 64 KB PSRAM ring version (committed); ring drained to SD at most every 50 ms in ≤4 KB chunks.
- `platformio.ini` — build config for the M5Stack Core2 target.
