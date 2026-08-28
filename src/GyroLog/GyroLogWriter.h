#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>
#include "Arduino_DebugUtils.h" // DEBUG_INFO / DEBUG_ERROR
#include "SdFat.h" // Adafruit SdFat (SdFat, FatFile, SdSpiConfig)
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
// "<clipname>.gcsv" file to the microSD card.
//
// The IMU is sampled at 1 kHz: a dedicated sampler task reads the MPU6886's
// output registers once per 1 ms tick (pinned to the real-time grid) and appends
// one dense row to a PSRAM ring buffer; a second (writer) task commits the ring
// to the card in batches, so a slow SD write never stalls the 1 kHz sampling.
// The "tscale" field is 1 ms, so Gyroflow's timeline is dense and accurate.
//
//   * begin(clipName, ...)  -> on record start: open the GCSV file, write the
//                              header, and start the sampler + writer tasks.
//   * poll()                -> no-op (the sampler task does all the sampling).
//   * end()                 -> on record stop: stop the tasks, drain the ring,
//                              close the file, and commit the directory entry.
//   * applySlateName(name)  -> after the real clip name is learned from the
//                              camera (via playback), rename the file.
//
// The SD card shares the VSPI bus with the M5GFX display, so SdFat is told
// not to re-initialise the bus (USER_SPI_BEGIN) and only toggles the SD's CS
// pin (GPIO4). See ensureSd().
class GyroLogWriter
{
public:
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

    // Configure the MPU6886 for OUTPUT-REGISTER polling at ~1 kHz: wake the
    // sensor, select the PLL gyro clock, set the DLPF/SMPLRT_DIV, and disable the
    // FIFO. We read the output registers (0x3B+) directly at 1 kHz from the
    // sampler task (not the FIFO), which keeps the I2C load trivial and the "t"
    // index dense by construction.
    void configurePolling();

    // Read the latest gyro+accel sample from the output registers (0x3B..0x48,
    // 14 bytes) and append one dense GCSV row to the ring, waking the writer.
    // Returns the number of rows appended (0 or 1). Used by the sampler task.
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

    // The sampler task's main loop: read the IMU output registers once per 1 ms
    // tick (pinned to the real-time grid) and append one dense row to the ring,
    // waking the writer as data arrives. Runs on its own FreeRTOS task.
    static void samplerTaskTrampoline(void* param);
    void samplerTask();
    void startSamplerTask();
    void stopSamplerTask();

    // ---- State ----
    State _state = State::Idle;
    Summary _summary;

    mutable SdFat _sd;
    FatFile _file;
    bool _sdReady = false;
    std::string _sdStatusMessage = "not mounted yet";

    // The ring buffer that decouples the IMU sampling from the SD write.
    PsramRing _ring;

    // ---- Writer task (decouples the SD write from the 1 kHz sampler) ----
    TaskHandle_t _writerTask = nullptr;
    SemaphoreHandle_t _ringMutex = nullptr;   // guards _ring (producer + consumer)
    SemaphoreHandle_t _dataSem = nullptr;     // wakes the writer when rows are pending
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

    // MPU6886 I2C address (7-bit). The M5 Core2's internal IMU sits at 0x68.
    static const uint8_t kImuAddr = 0x68;
    // I2C clock (Hz) for the 1 kHz output-register read. 1 MHz ("fast mode plus")
    // gives the read ~0.4 ms of headroom under the 1 ms tick. The M5Unified
    // In_I2C bus is shared with other sensors, but those are only touched on the
    // main loop (core 1) and the sampler runs on core 1 too at equal priority, so
    // a faster clock here doesn't disturb them.
    static const uint32_t kImuI2cHz = 1000000;
    uint32_t _i2cHz = kImuI2cHz;
};

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
extern const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount];

#endif // GYROLOGWRITER_H
