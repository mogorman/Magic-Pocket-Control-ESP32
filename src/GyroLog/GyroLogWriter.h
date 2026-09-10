#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>
#include <Wire.h> // TwoWire (shared I2C bus for the QMI8658)
#include "Arduino_DebugUtils.h" // DEBUG_INFO / DEBUG_ERROR
#include <FS.h>    // Arduino FS/File (used by SD_MMC)
#include <SD_MMC.h> // ESP32 4-bit SD_MMC (the 1.54's TF slot)
#include "SensorQMI8658.hpp" // Waveshare SensorLib QMI8658 driver
#include <freertos/FreeRTOS.h> // TaskHandle_t / SemaphoreHandle_t
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h> // ps_malloc (PSRAM-backed ring buffer)

// A ring buffer backed by a PSRAM allocation. SdFat's own RingBuf keeps its
// buffer in the object (internal RAM), so a large one (128 KB) overflows the
// ESP32's internal DRAM at link time. This version ps_malloc's the buffer so it
// can be as big as the PSRAM allows. It decouples the 1 kHz IMU sampler from the
// SD writer: the sampler appends rows to the ring, the writer commits them to the
// card in batches, so a slow (multi-ms) SD write never stalls the 1 kHz sampling.
class PsramRing
{
public:
    bool begin(File* file, size_t size)
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
    File* _file = nullptr;
    uint8_t* _buf = nullptr;
    size_t _size = 0;
    size_t _head = 0;
    size_t _tail = 0;
};

// GCSV (Gyroflow CSV) logger for the Waveshare ESP32-S3-Touch-AMOLED-2.16.
//
// Records the onboard QMI8658 gyro + accelerometer while a clip is being
// recorded on the connected Blackmagic camera, and writes a sidecar
// "<clipname>.gcsv" file to the microSD card.
//
// The IMU is sampled at 1 kHz: a dedicated sampler task reads the QMI8658's
// output registers once per 1 ms tick (pinned to the real-time grid) via a single
// 12-byte I2C burst read (accel+gyro are contiguous), and appends one dense row to
// a PSRAM ring buffer; a second (writer) task commits the ring to the card in
// batches, so a slow (multi-ms) SD write never stalls the 1 kHz sampling. The
// single burst read keeps the I2C time (~200 us) well under the 1 ms period, so we
// don't re-read the same sample on consecutive ticks. The "tscale" field is 1 ms,
// so Gyroflow's timeline is dense and accurate.
//
//   * begin(clipName, ...)  -> on record start: open the GCSV file, write the
//                              header, and start the sampler + writer tasks.
//   * poll()                -> no-op (the sampler task does all the sampling).
//   * end()                 -> on record stop: stop the tasks, drain the ring,
//                              close the file, and commit the directory entry.
//   * applySlateName(name)  -> after the real clip name is learned from the
//                              camera (via playback), rename the file.
//
// The SD card is on a DEDICATED SPI bus (not shared with the QSPI display), so
// SdFat drives it directly with its own CS pin (GPIO41). See ensureSd().
class GyroLogWriter
{
public:
    GyroLogWriter();
    ~GyroLogWriter();

    // The 24 possible orientation tokens: 6 axis permutations (X/Y/Z) x 4
    // sign combinations. Index 0..23 maps to a token via
    // GYROLOG_ORIENTATION_TOKENS.
    static const int kOrientationCount = 24;

    enum class State : uint8_t
    {
        Idle = 0,
        Recording = 1
    };

    // Summary of the most recently completed clip, shown on the Gyro Log screen.
    struct Summary
    {
        bool valid = false;
        std::string fileName;       // e.g. "A001C001_001.gcsv"
        std::string videoFileName; // e.g. "A001C001_001.braw"
        uint32_t durationMs = 0;
        std::string timecodeAtEnd;
        uint64_t fileSizeBytes = 0;
        uint64_t freeBytes = 0;
        uint64_t totalBytes = 0;
    };

    // Begin a new log: mount the SD card, open the GCSV file, write the header,
    // and start the sampler + writer tasks. Returns true on success.
    //   clipName  : base name without extension (e.g. "clip_0001")
    //   extension : the video extension (e.g. "braw"); recorded for the summary
    //   timecode  : the camera timecode at the moment recording started
    //   lensInfo  : the lens the camera reports; may be empty
    bool begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo = "");

    // No-op: the dedicated sampler task does all the IMU sampling. Kept as a
    // hook so the call site in loop() stays simple.
    void poll();

    // Read one live IMU sample for the calibration display: gyro in deg/s and
    // accel in g. Returns false if the sensor isn't up (nothing read).
    bool readImuLive(float& gx, float& gy, float& gz, float& ax, float& ay, float& az);

    // Bring the QMI8658 up (idempotent) so readImuLive() returns real data on the
    // calibration screen, which is shown while NOT recording. The sensor is left
    // powered up afterwards (it is only powered down at full device shutdown), so
    // this is a cheap no-op once it is up.
    void ensureImuUp();

    // Finalise the current log: stop the tasks, drain the ring, close the file,
    // and commit the directory entry to the card. Populates the summary. Returns
    // true if a log was active.
    bool end();

    // Set the timecode string to record as the "end" timecode in the summary.
    void setTimecodeAtEnd(const std::string& timecode) { _summary.timecodeAtEnd = timecode; }

    // Apply the real clip name (learned from the camera via playback) to the
    // just-finalised log: rename the file to "/<slateName>.gcsv". No-op if the
    // name is empty/placeholder.
    void applySlateName(const std::string& slateName, const std::string& extension);

    bool isRecording() const { return _state == State::Recording; }

    const Summary& getSummary() const { return _summary; }

    // ---- Shared I2C bus lock ----
    // The QMI8658, the CST816T touch, and the audio codec all share one I2C bus
    // (SDA=42/SCL=41). The IMU sampler task (core 1) and the main loop (core 1)
    // both drive that bus, and the FIFO burst read is a multi-tick I2C transaction
    // that a same-core preemption could corrupt mid-read. Every I2C transaction
    // (the FIFO read, the touch poll, the live IMU read) must hold this mutex for
    // the duration of the transaction. Created once in the constructor; take it with
    // i2cLock()/i2cUnlock() (or i2cGuard()).
    SemaphoreHandle_t i2cMutex() const { return _i2cMutex; }
    bool i2cLock(uint32_t timeoutMs = 200) const;
    void i2cUnlock() const;
    // RAII guard: takes the I2C mutex for the duration of a scope. Waits up to
    // 200 ms (a FIFO burst read is ~10-40 ms, so this is ample); if it can't get
    // the lock in that time it proceeds anyway (the I2C transaction is short and
    // the bus is shared, so a brief overlap is far less harmful than a deadlock).
    struct I2cGuard
    {
        SemaphoreHandle_t m;
        explicit I2cGuard(SemaphoreHandle_t mu) : m(mu)
        {
            if(m) xSemaphoreTake(m, pdMS_TO_TICKS(200));
        }
        ~I2cGuard()
        {
            if(m) xSemaphoreGive(m);
        }
    };

    // SD card status, for the on-screen diagnostic.
    bool sdReady() const { return _sdReady; }
    const std::string& sdStatusMessage() const { return _sdStatusMessage; }

    // Compute the free-space figure for the summary. This calls
    // freeClusterCount(), which on a large card walks the whole FAT (tens of
    // seconds). It must NOT run on the main loop (it would stall the UI and the
    // playback clip-name capture). Call it from a non-critical context instead;
    // the result is cached in the volume, so the main loop reads it cheaply.
    void refreshFreeSpace();

    // Filesystem helpers (used by the E2E test to verify a written file).
    bool fileExists(const std::string& path) const;
    uint64_t fileSize(const std::string& path) const;
    // Count the data rows (the "t,gx,gy,gz,ax,ay,az" sample lines) in a closed
    // GCSV file at `path`. Returns the row count, or -1 if it can't be opened.
    long countSamplesInFile(const std::string& path);
    // Scan a closed GCSV file's "t" index and report its health: first/last t,
    // row count, largest gap between consecutive t (a gap = dropped samples), and
    // whether t ever goes backwards (a reset). A clean file has first=0,
    // last=N-1, maxGap=1, no backwards steps.
    void analyzeTIndex(const std::string& path);

    // The measured sample rate in Hz (1/_tscale). The E2E test uses this to
    // compute the expected sample count for a given clip duration.
    float measuredRateHz() const { return (_tscale > 0.0f) ? (1.0f / _tscale) : 0.0f; }
    // Number of I2C read failures during the last recording (diagnostic).
    uint32_t i2cFailures() const { return _i2cFailures; }

    // ---- Orientation (calibration) ----
    int getOrientationIndex() const { return _orientationIndex; }
    void setOrientationIndex(int index);
    static const char* orientationToken(int index);
    void loadOrientation();

private:
    // Mount the SD card via SdFat (CS pin 4, shared VSPI bus). Sets
    // _sdReady / _sdStatusMessage. Returns true if a FAT volume is mounted.
    bool ensureSd();
    // Force the FAT directory cache (file sizes/pointers) to the card by
    // unmounting and remounting.
    void syncVolume();
    // Flush + close the open file.
    void closeFile();

    // Configure the QMI8658 for 1 kHz sampling: enable the gyro at 1024 dps and
    // the accelerometer at 8 g (both at their highest ODR). After configuring we
    // let the sensor settle ~100 ms (so the gyro's digital filter is warm) before
    // the sampler starts, so the first logged sample is already good (no zeroed
    // start). The sampler then reads the output registers at 1 kHz.
    void configurePolling();

    // Read one fresh gyro+accel sample from the QMI8658's output registers (a
    // single 12-byte I2C burst of the contiguous accel+gyro registers) and append
    // one dense GCSV row to the ring, waking the writer. Returns the number of
    // rows appended (0 or 1). Used by the sampler task, which calls it once per
    // 1 ms tick. The single burst read keeps the I2C time (~200 us) well under
    // the 1 ms period so we don't re-read the same sample on consecutive ticks.
    uint32_t pollOutputRegisters();

    // Drain as much of the ring as possible to the file. Runs on the writer task
    // (and once from end() to flush the tail). Uses a two-phase copy (fast
    // PSRAM->RAM copy under the mutex, then the slow SD write without the mutex)
    // so the sampler's ring appends are never blocked by a card write.
    void drainRing();

    // The writer task's main loop: wait for pending rows (or a stop request),
    // then drain the ring to the file. Runs on its own FreeRTOS task.
    static void writerTaskTrampoline(void* param);
    void writerTask();
    void startWriterTask();
    void stopWriterTask();

    // The sampler task's main loop: read the QMI8658 output registers once per
    // 1 ms tick (pinned to the real-time grid) via a single 12-byte I2C burst, and
    // append one dense GCSV row to the ring, waking the writer. The single burst
    // read keeps the I2C time well under the 1 ms period so we don't re-read the
    // same sample on consecutive ticks. Runs on its own FreeRTOS task.
    static void samplerTaskTrampoline(void* param);
    void samplerTask();
    void startSamplerTask();
    void stopSamplerTask();

    // ---- State ----
    State _state = State::Idle;
    Summary _summary;

    // The 1.54's TF slot is a 4-bit SD_MMC bus (no CS pin). We use the ESP32
    // core SD_MMC (an fs::SDMMCFS) rather than SdFat, which can't drive MMC.
    // SD_MMC is a global singleton, so _sd is a reference to it.
    fs::SDMMCFS& _sd = SD_MMC;
    File _file;
    bool _sdReady = false;
    std::string _sdStatusMessage = "not mounted yet";

    // The ring buffer that decouples the IMU sampling from the SD write.
    PsramRing _ring;

    // ---- Writer task (decouples the SD write from the 1 kHz sampler) ----
    TaskHandle_t _writerTask = nullptr;
    SemaphoreHandle_t _ringMutex = nullptr;   // guards _ring (producer + consumer)
    SemaphoreHandle_t _dataSem = nullptr;     // wakes the writer when rows are pending
    // Locks the shared I2C bus (see i2cMutex()). Created in the constructor.
    mutable SemaphoreHandle_t _i2cMutex = nullptr;
    volatile bool _writerStop = false;        // tells the writer task to exit
    uint32_t _lastWriteMicros = 0;            // micros() of the writer's last card write

    // ---- Sampler task (reads the IMU output registers at 1 kHz) ----
    TaskHandle_t _samplerTask = nullptr;
    volatile bool _samplerStop = false;      // tells the sampler task to exit

    // Writer batch policy: commit to the card once a decent chunk is buffered
    // (kMinWriteBytes) or a max interval has passed (kMaxWriteIntervalUs). We use
    // SMALL, FREQUENT batches (16 KB every ~100 ms) rather than one big batch: a
    // big SPI write is a long burst of SPI DMA/interrupt activity that stretches
    // the 1 kHz I2C sampler's reads past 1 ms (the source of sample loss). A 16 KB
    // burst is ~5x shorter, so each I2C read is far less likely to overlap an
    // active SPI write. The 128 KB PSRAM ring holds several batches with headroom.
    static const size_t kMinWriteBytes = 16 * 1024;
    static const uint32_t kMaxWriteIntervalUs = 100 * 1000;
    static const size_t kRingSize = 128 * 1024;

    // The name we started with, the video file name, and the .gcsv path.
    std::string _startedName;
    std::string _extension;
    std::string _videoFileName;
    std::string _lensInfo;
    std::string _gcsvPath;
    uint64_t _finalFileSizeBytes = 0;

    // Timing: t is the running sample index (the GCSV "t" value); _tscale is the
    // seconds-per-sample (1 ms for the 1 kHz poll).
    uint32_t _fifoSeq = 0;
    float _tscale = 0.001f;
    bool _fifoConfigured = false;

    // Diagnostics: count of I2C read failures during the last recording.
    volatile uint32_t _i2cFailures = 0;

    // The GCSV orientation token index (0..23), persisted in NVS.
    int _orientationIndex = 0;

    // NVS keys for persisting the orientation.
    static const char* kNvsNamespace;
    static const char* kNvsKeyOrientation;

    // QMI8658 I2C address (7-bit). The Waveshare board's IMU is strapped to the
    // "L" address 0x6B (the SensorLib names it QMI8658_L_SLAVE_ADDRESS).
    static const uint8_t kImuAddr = 0x6B;
    // The shared I2C bus pins (SDA/SCL). The QMI8658 driver is handed these so it
    // talks on the same bus as the touch and audio (the 1.54's I2C is 42/41).
    static const int kImuSda = 42;
    static const int kImuScl = 41;

    // The QMI8658 driver instance. It is created once (in configurePolling) and
    // reused for every recording; it does the one-time chip reset + sensor config.
    // The sampler reads the output registers via a single raw-Wire 12-byte burst
    // (see pollOutputRegisters) for the tightest possible I2C timing.
    SensorQMI8658 _qmi;
};

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
extern const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount];

#endif // GYROLOGWRITER_H
