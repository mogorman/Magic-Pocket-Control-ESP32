#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>
#include "FS.h" // for File / SD

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
// The IMU is sampled at ~1 kHz (the MPU6886's native ODR, polled as fast as
// the I2C bus allows). Data is quantised to fixed-point integers using the
// gscale/ascale constants below, exactly as Gyroflow expects.

class GyroLogWriter
{
public:
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

    const Summary& getSummary() const { return _summary; }

    // SD card status, for the on-screen diagnostic.
    //   true  = a card is present and the filesystem is mounted
    //   false = no card, or the mount failed
    bool sdReady() const { return _sdReady; }
    // Human-readable message about the last SD attempt (e.g. "no card",
    // "mount failed", "ready"). Empty until the first attempt.
    const std::string& sdStatusMessage() const { return _sdStatusMessage; }

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

    // The GCSV file, as a POSIX VFS file descriptor. We deliberately do NOT use
    // the Arduino File / newlib stdio (fwrite) path: newlib's FILE* buffering
    // corrupts the internal heap when exercised during 1 kHz I2C sampling (the
    // __sfvwrite_r -> memmove fault we kept hitting). Instead we open the file
    // with the VFS open() syscall and write with the VFS write() syscall, which
    // routes straight to the FatFs f_write handler with no newlib FILE buffer.
    // The data is written in small chunks from a dedicated writer task so the
    // 1 kHz I2C sampling on the loop task never does SD work itself.
    int _fd = -1;

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

    // The final on-card size of the .gcsv file, captured just before the writer
    // task closes it (the FIL handle is then freed, so we can't query it after).
    uint64_t _finalFileSizeBytes = 0;

    // Timing: t is the elapsed ms since the clip started (real, micros-based so
    // it stays accurate even if a sample is dropped).
    uint32_t _tMs = 0;
    uint32_t _lastSampleMicros = 0;
    uint32_t _startMicros = 0; // micros() at the moment the log started
    uint32_t _lastDrainMicros = 0; // micros() of the last ring drain (rate-limits SD writes)

    // The PSRAM ring buffer that decouples the 1 kHz IMU read from the slower
    // SD write. Rows are appended here by the sampler (loop task) and drained
    // to the file by the dedicated writer task.
    static const size_t kRingSize = 64 * 1024; // 64 KB
    // Size of the internal-DRAM chunk buffer used to copy data out of the ring
    // before the SD write. Kept small (4 KB) so the internal allocation always
    // succeeds and the write reads from a stable internal buffer.
    static const size_t kChunkSize = 4096;
    char* _ring = nullptr;
    size_t _ringWrite = 0;  // next byte to write
    size_t _ringRead = 0;   // next byte to read
    size_t _ringCount = 0; // bytes currently buffered

    // The GCSV orientation token index (0..23), persisted in NVS.
    int _orientationIndex = 0;

    // MPU6886 I2C address (7-bit). The M5 Core2's internal IMU sits at 0x68.
    static const uint8_t kImuAddr = 0x68;

    // SD card status (for the on-screen diagnostic).
    bool _sdReady = false;
    std::string _sdStatusMessage;

    // The summary of the most recently completed clip.
    Summary _summary;

    // NVS keys for persisting the orientation.
    static const char* kNvsNamespace;
    static const char* kNvsKeyOrientation;

    // Drain as much of the ring buffer as possible to the file. Called from
    // poll() (rate-limited) and once more from end() to flush the tail.
    void drainRing();

    // Close the file descriptor. Safe to call when no file is open.
    void closeFile();

    // Allocate the ring buffer in PSRAM. Returns true on success.
    bool allocRing();

    // Make sure the SD card is mounted (CS pin GPIO4). Sets _sdReady and
    // _sdStatusMessage. Returns true if the card is ready.
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
