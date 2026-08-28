#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>
#include "SdFat.h" // Adafruit SdFat (FatFile, SdSpiConfig)
#include "RingBuf.h" // Adafruit SdFat RingBuf (decouples 1 kHz sampling from SD writes)
#include <freertos/FreeRTOS.h> // TaskHandle_t / SemaphoreHandle_t (writer task)
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h> // ps_malloc (PSRAM-backed ring buffer)

// A ring buffer backed by a PSRAM allocation, with the same interface the
// writer uses from SdFat's RingBuf (begin/write/writeOut/bytesUsed). SdFat's
// RingBuf keeps its buffer in the object (internal RAM), so a large one (e.g.
// 128 KB for 80 KB write chunks) overflows the ESP32's internal DRAM at link
// time. This version ps_malloc's the buffer so it can be as big as the PSRAM
// allows. Used to decouple the IMU sampler from the SD writer task.
class PsramRing
{
public:
    bool begin(FatFile* file, size_t size)
    {
        _file = file;
        _size = size;
        _buf = (uint8_t*)ps_malloc(size);
        if(!_buf)
            return false;
        _head = _tail = 0;
        return true;
    }
    // Append up to `count` bytes; drops (returns 0) if the ring can't hold them.
    size_t write(const void* buf, size_t count)
    {
        if(freeSpace() < count)
            return 0;
        const uint8_t* src = (const uint8_t*)buf;
        size_t n = count < (_size - _head) ? count : (_size - _head);
        memcpy(_buf + _head, src, n);
        _head = (_head + n) % _size;
        if(n < count)
        {
            memcpy(_buf, src + n, count - n);
            _head = count - n;
        }
        return count;
    }
    // Commit up to `count` buffered bytes to the file; returns bytes written.
    size_t writeOut(size_t count)
    {
        size_t avail = used();
        count = count < avail ? count : avail;
        size_t n = count < (_size - _tail) ? count : (_size - _tail);
        _file->write(_buf + _tail, n);
        _tail = (_tail + n) % _size;
        if(n < count)
        {
            _file->write(_buf, count - n);
            _tail = count - n;
        }
        return count;
    }
    // Copy up to `count` buffered bytes out of the ring into a caller-provided
    // buffer (a fast PSRAM->RAM memcpy) and advance the ring tail, WITHOUT doing
    // the (slow) SD write. This lets the writer hold the ring mutex only for the
    // fast copy, then do the slow SD write on the local buffer without the mutex --
    // so the sampler's ring appends are never blocked by a card write. Returns the
    // number of bytes copied.
    size_t copyOut(uint8_t* dst, size_t count)
    {
        size_t avail = used();
        count = count < avail ? count : avail;
        size_t n = count < (_size - _tail) ? count : (_size - _tail);
        memcpy(dst, _buf + _tail, n);
        _tail = (_tail + n) % _size;
        if(n < count)
        {
            memcpy(dst + n, _buf, count - n);
            _tail = count - n;
        }
        return count;
    }
    size_t used() const { return (_head >= _tail) ? (_head - _tail) : (_size - _tail + _head); }
    size_t bytesUsed() const { return used(); } // alias matching SdFat RingBuf's name
    size_t freeSpace() const { return _size - used(); }
private:
    FatFile* _file = nullptr;
    uint8_t* _buf = nullptr;
    size_t _size = 0;
    size_t _head = 0;
    size_t _tail = 0;
};

// GCSV (Gyroflow CSV) logger for the M5Stack Core2.
//
// Records the onboard MPU6886 gyro + accelerometer while a clip is being
// recorded on the connected Blackmagic camera, and writes a sidecar
// "<clipname>.gcsv" file to the microSD card. The file name matches the
// video file (e.g. "A001C001_001.braw" -> "A001C001_001.gcsv").
//
// The GCSV format is documented at:
//   https://docs.gyroflow.xyz/app/technical-details/gcsv-format
//
// The IMU is sampled via the MPU6886's internal FIFO (drained in bursts), so
// that a stall on the shared SPI bus (SD card write) never loses samples -- the
// FIFO holds them until the I2C bus is free again. The sensor's true rate is
// measured at start and declared in the GCSV "tscale" field so Gyroflow
// resamples correctly. Data is quantised to fixed-point integers using the
// gscale/ascale constants below, exactly as Gyroflow expects.

class GyroLogWriter
{
public:
    // Destructor: stop the writer task and delete the FreeRTOS objects it uses.
    // (The writer is a global singleton and is never actually destroyed, but this
    // keeps the class well-behaved if it ever is.)
    ~GyroLogWriter();

    // The GCSV orientation token. This must match how the MPU6886 is physically
    // mounted on the Core2. It is calibrated at runtime (see the Gyro Log
    // screen) and persisted in NVS, so it survives reboots.
    //
    // The 24 possible tokens are the 6 axis permutations (X/Y/Z) x 4 sign
    // combinations (x, y, z each +/-). The index 0..23 maps to a token via
    // GYROLOG_ORIENTATION_TOKENS.
    static const int kOrientationCount = 24;

    enum class State : uint8_t
    {
        Idle = 0,
        Recording = 1,
        Finalizing = 2
    };

    // Summary of the most recently completed clip, shown on the Gyro Log screen.
    struct Summary
    {
        bool valid = false;
        std::string fileName;       // e.g. "A001C001_001.gcsv"
        std::string videoFileName;  // e.g. "A001C001_001.braw"
        uint32_t durationMs = 0;
        std::string timecodeAtEnd;
        uint64_t fileSizeBytes = 0;
        uint64_t freeBytes = 0;
        uint64_t totalBytes = 0;
    };

    // Begin a new log. Returns true on success (file opened, header written).
    //   clipName  : base name without extension (e.g. "A001C001_001" or "clip_0001")
    //   extension : the video extension (e.g. "braw", "mov")
    //   timecode  : the camera timecode at the moment recording started
    //   lensInfo  : the lens the camera reports (e.g. "Canon EF-S 18-55mm ...");
    //               recorded in the GCSV "lens_info" field. May be empty.
    bool begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo = "");

    // Sync the ESP32 system clock (and the M5 RTC) from the camera's
    // timecode. The timecode gives a reliable time-of-day (HH:MM:SS); we map
    // it onto the date we are told to use (the clip's date, or a default year
    // when that is not yet known) so the resulting UNIX timestamp is a real,
    // current value. Returns true if the clock was set.
    bool syncRtcFromTimecode(const std::string& timecode, int year);

    // Poll the IMU and append a sample if ~1 ms has elapsed since the last
    // sample. Call this every loop() iteration while recording.
    void poll();

    // Finalise the current log: flush, close, and (if a slate name arrived
    // after we started with a generic name) rename the file. Populates the
    // summary. Returns true if a log was active and finalised.
    bool end();

    // Set the timecode string to record as the "end" timecode in the summary
    // (call just before end()).
    void setTimecodeAtEnd(const std::string& timecode) { _summary.timecodeAtEnd = timecode; }

    // Apply a real clip name to a *finalised* log. Call after end() with the
    // slate name the camera reported (the "previous" clip's slate, i.e. the
    // clip that was just recorded). This renames the .gcsv file and rewrites
    // the "videofilename" header line so both match the real video file.
    //
    // It is deliberately done only after the file has been closed (in end()),
    // never while the IMU is still being sampled, so the 1 kHz sample stream is
    // never interrupted and no data can be lost to a mid-recording rename.
    // No-op if the name is empty, a placeholder, or already the current name.
    void applySlateName(const std::string& slateName, const std::string& extension);

    State state() const { return _state; }
    bool isRecording() const { return _state == State::Recording; }

    // The measured IMU sample rate in Hz (1/_tscale), measured at begin(). The
    // end-to-end test uses this to compute the expected sample count for a
    // given clip duration.
    float measuredRateHz() const { return (_tscale > 0.0f) ? (1.0f / _tscale) : 0.0f; }

    // Diagnostics from the last recording: how many FIFO-data I2C reads failed
    // (a non-zero count at a high I2C clock means the clock is too fast for the
    // sensor) and how many times the FIFO overflowed (we fell behind draining).
    uint32_t i2cFailures() const { return _i2cFailures; }
    uint32_t fifoOverflows() const { return _fifoOverflows; }

    // Set the I2C clock (Hz) used for the FIFO drain. The E2E test uses this to
    // sweep a few clock values and find the reliability/throughput sweet spot.
    // Takes effect on the next recording (drainFifoOnce reads _i2cHz each pass).
    void setI2cHz(uint32_t hz) { _i2cHz = hz; }

    // Diagnostic: sweep DLPF_CFG x FCHOICE_B and log the measured FIFO rate for
    // each combo, to find a config that yields ~1 kHz (drainable over I2C)
    // instead of the clone's default ~3.8 kHz. Leaves the sensor in a clean
    // (non-FIFO) state; call configureFifo() afterwards for a real recording.
    void sweepFifoRate();

    const Summary& getSummary() const { return _summary; }

    // SD card status, for the on-screen diagnostic.
    //   true  = a card is present and the filesystem is mounted
    //   false = no card, or the mount failed
    bool sdReady() const { return _sdReady; }
    // Human-readable message about the last SD attempt (e.g. "no card",
    // "mount failed", "ready"). Empty until the first attempt.
    const std::string& sdStatusMessage() const { return _sdStatusMessage; }

    // Count the data rows (the "t,gx,gy,gz,ax,ay,az" sample lines) in a GCSV
    // file at `path` (e.g. "/clip_0001.gcsv"). The file must be closed; this
    // opens it read-only on the already-mounted volume, counts lines that start
    // with a digit (the sample rows), and closes it. Returns the row count, or
    // -1 if the file could not be opened. Used by the end-to-end test to verify
    // the recorded sample count.
    long countSamplesInFile(const std::string& path);

    // Dump the first `n` bytes of a GCSV file to the debug log (for verifying
    // the header + first sample rows are well-formed). Used by the E2E test.
    void dumpFileHead(const std::string& path, size_t n);

    // Scan a closed GCSV file's "t" index and report its health for Gyroflow:
    // the first and last t, the count of rows, the largest gap between
    // consecutive t values (a gap = dropped samples), and whether t ever goes
    // backwards (a reset, which would corrupt Gyroflow's timeline). A clean file
    // has first=0, last=N-1, maxGap=1, and no backwards steps. Used by the E2E
    // test to confirm the timeline is usable, not just that samples exist.
    void analyzeTIndex(const std::string& path);

    // ---- Orientation (calibration) ----
    // Returns the current orientation token index (0..23).
    int getOrientationIndex() const { return _orientationIndex; }
    // Set the orientation token index (0..23) and persist it to NVS.
    void setOrientationIndex(int index);
    // The GCSV orientation string for a given index (e.g. "YxZ").
    static const char* orientationToken(int index);
    // Load the persisted orientation index from NVS (call once at startup).
    void loadOrientation();

    // ---- IMU access (used by the calibration screen) ----
    // Read the current gyro (rad/s) and accel (g). Returns true on success.
    //
    // This reads the MPU6886 *directly* over I2C (a single 14-byte burst of the
    // accel+gyro data registers) rather than going through M5Unified's
    // getGyro()/getAccel(), which throttle re-reads to ~10 Hz (a 256 us gate
    // around a 15-byte read). Reading directly lets us sample at the sensor's
    // true 1 kHz ODR.
    bool readImu(float* gx, float* gy, float* gz, float* ax, float* ay, float* az);

private:
    State _state = State::Idle;

    // The SD card / filesystem, driven by Adafruit SdFat (not the Arduino SD /
    // VFS / newlib-stdio path, which corrupted the internal heap when exercised
    // during 1 kHz I2C sampling). SdFat talks to the card over SPI directly and
    // writes through its own FatFile, with no newlib FILE* buffering.
    SdFat _sd;

    // The GCSV file we are writing (only valid while recording/finalizing).
    FatFile _file;

    // The ring buffer that decouples the IMU sampling from the SD write,
    // modeled on the Adafruit SdFat high-speed-logging pattern (TeensySdioLogger):
    // the sampler writes GCSV rows into the ring, and a drain calls writeOut()
    // which commits the buffered bytes to the FatFile. It's a PSRAM-backed ring
    // (128 KB) so the writer can batch large 80 KB chunks to the card without the
    // ring overflowing internal RAM at link time.
    //
    // The sampler (loop task) and the writer task both touch this ring, so all
    // access is serialized by _ringMutex (the ring's indices are not thread-safe
    // across tasks).
    PsramRing _ring;

    // ---- Writer task (decouples the SD write from the 1 kHz sampler) ----
    // The sampler on the loop task only appends rows to the ring; a dedicated
    // FreeRTOS task (on the other core) does the actual FatFile::write(). This
    // is what lets the sampler hold a true 1 kHz cadence: a card write takes
    // tens of ms, and if it ran on the loop task it would stall the sampler
    // (measured: 76% capture with a same-task write vs 100% without).
    TaskHandle_t _writerTask = nullptr;
    SemaphoreHandle_t _ringMutex = nullptr;   // guards _ring (producer + consumer)
    SemaphoreHandle_t _dataSem = nullptr;     // wakes the writer when rows are pending
    volatile bool _writerStop = false;        // tells the writer task to exit
    uint32_t _lastWriteMicros = 0;           // micros() of the writer's last card write (batch rate-limit)

    // ---- Sampler task (drains the MPU6886 FIFO) ----
    // The MPU6886's 1 KB FIFO only buffers ~25 ms of samples at its ~2.87 kHz
    // rate. The main loop can't drain it that fast (it does UI/BLE work), so the
    // FIFO overflowed and we lost ~86% of samples. The fix: a dedicated
    // high-priority task that drains the FIFO every ~1-2 ms (well within the
    // 25 ms window) and appends rows to the ring. The writer task (above) then
    // commits the ring to the card. Pinned to core 1 (with the loop task) at a
    // higher priority so it preempts the UI to drain the FIFO promptly; the I2C
    // read and the SPI card write (core 0) stay on separate cores.
    TaskHandle_t _samplerTask = nullptr;
    volatile bool _samplerStop = false;      // tells the sampler task to exit
    // Bounded FIFO-drain guard: max I2C reads per poll() pass, so a stuck FIFO
    // count can't spin the sampler. (The FIFO count normally decreases as we
    // read, but a misbehaving clone could leave it high.)
    static const int kMaxFifoReadsPerPass = 8;

    // Writer batch policy: only commit to the card once a decent chunk is
    // buffered (kMinWriteBytes) or a max interval has passed (kMaxWriteIntervalUs).
    // This keeps the SPI transaction rate low (a few per 10 ms instead of ~1000/
    // s), which is what lets the I2C IMU read on the other core stay undisturbed.
    // 80 KB is the practical max batch (240 KB caused intermittent shared-SPI-bus
    // assert crashes); the 128 KB PSRAM ring holds it with headroom.
    static const size_t kMinWriteBytes = 80 * 1024;
    static const uint32_t kMaxWriteIntervalUs = 50 * 1000;
    // Size of the PSRAM ring buffer (bytes). Must be >= kMinWriteBytes.
    static const size_t kRingSize = 128 * 1024;

    // The name we started with (may be a generic "clip_NNNN").
    std::string _startedName;
    std::string _extension;

    // The video file name to record in the GCSV "videofilename" field. Defaults
    // to "<startedName>.<extension>" and is updated by applySlateName() after
    // the file is finalised, when a real slate name becomes available.
    std::string _videoFileName;

    // The lens the camera reports (GCSV "lens_info" field). Recorded in the
    // header at begin(); empty if the camera didn't report one.
    std::string _lensInfo;

    // The GCSV "timestamp" (UNIX epoch) we wrote in the header at begin(), and
    // the .gcsv file path we started with. Both are remembered so that, once the
    // real clip name (and therefore the real date) is known, we can correct the
    // header timestamp and the file's FAT modification time.
    long _timestampEpoch = 0;
    std::string _gcsvPath;

    // The final on-card size of the .gcsv file, captured just before the file is
    // closed (FatFile::fileSize()).
    uint64_t _finalFileSizeBytes = 0;

    // Timing: t is the elapsed ms since the clip started (real, micros-based so
    // it stays accurate even if a sample is dropped).
    uint32_t _tMs = 0;
    uint32_t _lastSampleMicros = 0;
    uint32_t _startMicros = 0; // micros() at the moment the log started
    uint32_t _lastDrainMicros = 0; // micros() of the last ring drain (rate-limits SD writes)

    // ---- FIFO-based sampling (the fix for data loss on SD-write stalls) ----
    // The MPU6886's internal FIFO buffers samples. Instead of reading the latest
    // sample from the output registers every 1 ms (which loses the in-between
    // samples whenever the shared SPI bus stalls the I2C read), poll() DRAINS the
    // FIFO: every sample that accumulated while we were busy is read out, so
    // nothing is lost. Each drained packet is one GCSV row.
    //
    // The "t" field is a running packet index (0, 1, 2, ...) -- NOT wall-clock ms
    // -- because the sensor's true rate is not 1 kHz. The GCSV "tscale" field
    // (measured at begin()) tells Gyroflow the real seconds-per-sample, so it
    // resamples to the video rate correctly.
    uint32_t _fifoSeq = 0;          // running packet index (the GCSV "t" value)
    uint8_t _fifoBuf[1024];         // scratch for a FIFO read (up to the full 1 KB FIFO = 73 packets)
    // Diagnostics: count of I2C read failures and FIFO overflows during the last
    // recording. Exposed via i2cFailures()/fifoOverflows() for the E2E test to
    // report (a non-zero I2C-failure count at a higher clock means the clock is
    // too fast for the sensor).
    volatile uint32_t _i2cFailures = 0;
    volatile uint32_t _fifoOverflows = 0;
    float _tscale = 0.001f;         // measured seconds-per-sample (written to the header)
    bool _fifoConfigured = false;  // true once begin() set up the FIFO
    // ---- Decimation to a clean, gap-free output rate (e.g. ~1 kHz) ----
    // The sensor's FIFO runs at ~3.8 kHz, faster than the I2C bus can drain
    // losslessly, so we DECIMATE: keep every Nth packet and write a dense
    // t=0,1,2,... index at the decimated rate. N is chosen so the output is
    // ~1 kHz (3.8k/4 = ~950 Hz). _decimatePacket is a running counter over ALL
    // packets read (not just the ones we keep) so the sampling phase is stable
    // and the output rate is exactly sourceRate/N.
    static const uint32_t kDecimateN = 4; // keep 1 of every 4 packets -> ~950 Hz
    uint32_t _decimatePacket = 0;          // running count of all packets read

    // The GCSV orientation token index (0..23), persisted in NVS.
    int _orientationIndex = 0;

    // MPU6886 I2C address (7-bit). The M5 Core2's internal IMU sits at 0x68.
    static const uint8_t kImuAddr = 0x68;

    // I2C clock (Hz) for draining the MPU6886 FIFO. 400 kHz is the datasheet
    // "fast mode" limit, but the FIFO data read is the throughput bottleneck at
    // the sensor's ~3.8 kHz rate (we need to move ~53 KB/s and 400 kHz I2C tops
    // out at ~50 KB/s). 1 MHz ("fast mode plus") gives ~2.5x headroom. The
    // M5Unified In_I2C bus is shared with other sensors, but they're only touched
    // on the main loop (core 1) and the FIFO drain runs on the sampler task
    // (core 0), so a faster clock here doesn't disturb them. The E2E clock sweep
    // (400k-1.5M) showed 1.5 MHz gives the best capture with the fewest I2C read
    // failures on this clone, so that's the default. (If a different board's
    // sensor misbehaves at 1.5 MHz, drop this back to 1000000 or 400000.)
    static const uint32_t kImuI2cHz = 600000;
    // The I2C clock actually used for the FIFO drain. Defaults to kImuI2cHz; the
    // E2E test can override it via setI2cHz() to sweep for the best value.
    uint32_t _i2cHz = kImuI2cHz;

    // SD card status (for the on-screen diagnostic).
    bool _sdReady = false;
    std::string _sdStatusMessage;

    // The summary of the most recently completed clip.
    Summary _summary;

    // NVS keys for persisting the orientation.
    static const char* kNvsNamespace;
    static const char* kNvsKeyOrientation;

    // Drain as much of the ring buffer as possible to the file (RingBuf::writeOut).
    // Runs on the writer task (and once from end() on the loop task to flush the
    // tail). Takes _ringMutex around the ring access.
    void drainRing();

    // Configure the MPU6886's FIFO for sampling: set the clock source, ODR
    // divider, DLPF, enable the FIFO for gyro+accel, and reset it. Then measure
    // the sensor's true sample rate over a short window and store it in _tscale.
    // Called from begin() before the first sample.
    void configureFifo();

    // Configure the MPU6886 for OUTPUT-REGISTER polling (the 1 kHz sampling mode):
    // wake the sensor, set the clock source, and set the DLPF/SMPLRT_DIV so the
    // output registers (0x3B+) refresh at a known rate. We then poll those
    // registers at 1 kHz from the sampler task. This avoids the FIFO's ~3.8 kHz
    // rate (which the I2C bus can't drain losslessly) -- at 1 kHz the I2C load is
    // trivial (~14 KB/s) and the "t" index is dense by construction.
    void configurePolling();

    // Read the latest gyro+accel sample from the output registers (0x3B..0x48,
    // 14 bytes: accel 6, temp 2, gyro 6) and append one dense GCSV row to the
    // ring. Returns the number of rows appended (0 or 1). Used by the sampler
    // task's 1 kHz poll loop.
    uint32_t pollOutputRegisters();

    // The writer task's main loop: wait for pending rows (or a stop request),
    // then drain the ring to the file. Runs on its own FreeRTOS task.
    static void writerTaskTrampoline(void* param);
    void writerTask();

    // Start / stop the writer task. startWriterTask() is called from begin();
    // stopWriterTask() from end() (signals stop, then joins the task).
    void startWriterTask();
    void stopWriterTask();

    // The sampler task's main loop: continuously drain the MPU6886 FIFO into the
    // ring (fast enough to never overflow the 1 KB FIFO), waking the writer as
    // data arrives. On a stop request it does a final FIFO drain and exits.
    // Runs on its own high-priority FreeRTOS task.
    static void samplerTaskTrampoline(void* param);
    void samplerTask();

    // Start / stop the sampler task. startSamplerTask() is called from begin();
    // stopSamplerTask() from end() (signals stop, does a final drain, joins).
    void startSamplerTask();
    void stopSamplerTask();

    // Drain the MPU6886 FIFO once, appending each packet to the ring. Shared by
    // the sampler task's loop and its final drain. Bounded so a stuck FIFO count
    // can't spin. Returns the number of packets drained this pass.
    uint32_t drainFifoOnce();

    // Close the FatFile and commit it to the card. Safe to call when no file is open.
    void closeFile();

    // Make sure the SD card is mounted via SdFat (CS pin GPIO4). Sets _sdReady
    // and _sdStatusMessage. Returns true if the card is ready.
    bool ensureSd();

    // Force the FAT volume's cached directory entries to be written to the
    // card (unmount + remount). Call after closing a file so its size is not
    // read back as 0 bytes on a computer.
    void syncVolume();

    // Rewrite the "videofilename" header line of a closed .gcsv file at `path`
    // to `newVideoFileName`. Returns true on success.
    bool rewriteVideoFileName(const std::string& path, const std::string& newVideoFileName);

    // Rewrite the "timestamp" header line of a closed .gcsv file at `path` to
    // `epoch` (UNIX seconds). Returns true on success.
    bool rewriteTimestamp(const std::string& path, long epoch);

    // Set the FAT modification time of the file at `path` to `epoch` (UNIX
    // seconds). Returns true on success.
    bool setFileMtime(const std::string& path, long epoch);

    // Persist / load the orientation index via NVS.
    void saveOrientation();
};

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
// Order: for each of the 6 permutations of (x,y,z) axes, 4 sign combos.
extern const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount];

#endif // GYROLOGWRITER_H
