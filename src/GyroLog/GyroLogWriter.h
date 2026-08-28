#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>
#include "Arduino_DebugUtils.h" // DEBUG_INFO / DEBUG_ERROR
#include "SdFat.h" // Adafruit SdFat (SdFat, FatFile, SdSpiConfig)

// SD-backed gyro log writer.
//
// This is the DATA side: it mounts the Core2's microSD card via SdFat and, for
// each recorded clip, writes a sidecar data file next to where the GCSV data
// will eventually go.
//
//   * begin(clipName, ...)  -> on record start: create the data file and write
//                              its header.
//   * poll()               -> on each main-loop tick while recording: append one
//                              sample's worth of data at the real 1 kHz rate.
//   * end()                -> on record stop:  close the file and force the
//                              FAT directory entry to the card.
//   * applySlateName(name) -> after the real clip name is learned from the
//                              camera (via playback), rename the file to
//                              "/<realName>.<ext>" and rewrite its contents to
//                              match the real name.
//
// DATA MOCK (GYRO_MOCK_DATA, default 1): the real IMU sampling is not ported
// yet, so poll() writes a *mock* GCSV row at the same rate and (approximately)
// the same byte size the real 1 kHz sampler will produce, so we can validate the
// SD write path under realistic load: does the card sustain the write rate, and
// does the finished file have exactly the expected size and a complete,
// well-formed body? With GYRO_MOCK_DATA=0 the data side is a no-op (the file
// just holds a "hello world" payload) for the earlier bring-up tests.
//
// The SD card shares the VSPI bus with the M5GFX display, so SdFat is told
// not to re-initialise the bus (USER_SPI_BEGIN) and only toggles the SD's CS
// pin (GPIO4). See ensureSd().
#ifndef GYRO_MOCK_DATA
#define GYRO_MOCK_DATA 1
#endif
class GyroLogWriter
{
public:
    // The 24 possible orientation tokens: 6 axis permutations (X/Y/Z) x 4
    // sign combinations. Index 0..23 maps to a token via
    // GYROLOG_ORIENTATION_TOKENS. (Orientation persistence is a later step;
    // the index is kept in RAM only for now.)
    static const int kOrientationCount = 24;

    // Summary of the most recently completed clip, shown on the Gyro Log screen.
    struct Summary
    {
        bool valid = false;
        std::string fileName;       // e.g. "A001C001_001.txt"
        std::string videoFileName; // e.g. "A001C001_001.braw"
        uint32_t durationMs = 0;
        std::string timecodeAtEnd;
        uint64_t fileSizeBytes = 0;
        uint64_t freeBytes = 0;
        uint64_t totalBytes = 0;
        // Mock-data bookkeeping (only meaningful when GYRO_MOCK_DATA is on): the
        // number of data rows written and the header byte count, so the expected
        // file size (header + rows * kMockRowBytes) can be checked exactly.
        uint32_t mockRows = 0;
        uint32_t mockHeaderBytes = 0;
    };

    // Begin a new log: mount the SD card and create "/<clipName>.txt" with a
    // "hello world" payload. Returns true on success.
    //   clipName  : base name without extension (e.g. "clip_0001")
    //   extension : the video extension (e.g. "braw"); recorded for the summary
    //   timecode  : the camera timecode at the moment recording started
    //   lensInfo  : the lens the camera reports; may be empty
    bool begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo = "");

    // Poll the IMU. With GYRO_MOCK_DATA this appends one mock GCSV row per
    // ~1 ms tick (the real 1 kHz rate) to the open data file. With
    // GYRO_MOCK_DATA=0 it is a no-op.
    void poll();

    // Finalise the current log: close the file and commit the directory entry
    // to the card. Populates the summary. Returns true if a log was active.
    bool end();

    // Set the timecode string to record as the "end" timecode in the summary.
    void setTimecodeAtEnd(const std::string& timecode) { _summary.timecodeAtEnd = timecode; }

    // Apply the real clip name (learned from the camera via playback) to the
    // just-finalised log: rename the file to "/<slateName>.txt" and rewrite its
    // contents to match the real name. No-op if the name is empty/placeholder.
    void applySlateName(const std::string& slateName, const std::string& extension);

    bool isRecording() const { return _state == State::Recording; }

    const Summary& getSummary() const { return _summary; }

    // SD card status, for the on-screen diagnostic.
    bool sdReady() const { return _sdReady; }
    const std::string& sdStatusMessage() const { return _sdStatusMessage; }

    // Compute the free-space figure for the summary. This calls
    // freeClusterCount(), which on a FAT32 card walks the whole FAT (tens of
    // seconds). It must NOT run on the main loop (it would stall the UI and the
    // playback clip-name capture). Call it from a non-critical context (e.g.
    // the BLE notify thread) instead; the result is cached in the volume, so
    // the main loop can then read summary.freeBytes cheaply.
    void refreshFreeSpace();

    // Filesystem helpers (used by the E2E test to verify a written file).
    // Returns true if a file exists at `path` (e.g. "/clip_0001.txt").
    bool fileExists(const std::string& path) const;
    // Returns the size in bytes of the file at `path`, or 0 if it can't be
    // opened.
    uint64_t fileSize(const std::string& path) const;

    // ---- MPU6886 FIFO self-test ----
    // Configure the sensor's FIFO (gyro+accel, 10-byte packets) at the given
    // output-data-rate divider (0 = no divider = the sensor's native rate).
    // Returns true if the FIFO was enabled.
    bool configureFifo(int smplrtDiv);
    // For ~3 seconds, repeatedly read the FIFO count register and drain the
    // FIFO, counting how many 10-byte packets arrive. Logs the measured
    // packets/second (the real FIFO sample rate) and a few sample values.
    // Returns the measured rate in Hz (0 on failure).
    float measureFifoRate();
    // Re-open the file at `path` read-only and verify its body is complete and
    // well-formed: it must contain exactly `expectedRows` data rows (each a
    // "t,gx,gy,gz,ax,ay,az" line), the final row's "t" index must equal
    // expectedRows-1 (i.e. no rows were dropped), and the file must end with a
    // newline. Returns true only if all of that holds. Used by the E2E test to
    // confirm the file was written completely, not just that it has the right
    // size.
    bool verifyFileComplete(const std::string& path, uint32_t expectedRows) const;

    // ---- Orientation (calibration) ----
    int getOrientationIndex() const { return _orientationIndex; }
    void setOrientationIndex(int index);
    static const char* orientationToken(int index);
    void loadOrientation();

private:
    enum class State : uint8_t
    {
        Idle = 0,
        Recording = 1
    };

    // Mount the SD card via SdFat (CS pin 4, shared VSPI bus). Sets
    // _sdReady / _sdStatusMessage. Returns true if a FAT volume is mounted.
    bool ensureSd();
    // Force the FAT directory cache (file sizes/pointers) to the card by
    // unmounting and remounting.
    void syncVolume();
    // Write the "hello world" payload for a given clip name to a file at `path`.
    bool writeHelloWorld(const std::string& path, const std::string& clipName);
#if GYRO_MOCK_DATA
    // Write the GCSV header (the "GYROFLOW IMU LOG" block) to the open data
    // file. Returns the number of bytes written (0 on failure).
    int writeGcsvHeader(const std::string& timecode, const std::string& videoFileName);
#endif

    State _state = State::Idle;
    Summary _summary;

    mutable SdFat _sd;
    FatFile _file;
    bool _sdReady = false;
    std::string _sdStatusMessage = "not mounted yet";

    // The clip name the log was started with, and the path of the open file.
    std::string _startedName;
    std::string _extension;
    std::string _gcsvPath;
    uint64_t _finalFileSize = 0;

    int _orientationIndex = 0;

#if GYRO_MOCK_DATA
    // Mock-data state. The file is a real GCSV: a fixed header followed by one
    // row per 1 ms tick. The mock row is a fixed-width string so the finished
    // file size is exactly predictable (header + rows * kMockRowBytes).
    // Row format: "000000,0,0,0,0,0,0\n" = 19 bytes.
    static const size_t kMockRowBytes = 19;
    uint32_t _mockSeq = 0;          // the "t" index for the next row (0,1,2,...)
    uint32_t _mockLastTick = 0;    // millis() of the last row written
    uint32_t _mockRowsWritten = 0; // total rows written this clip (for the summary)
    uint32_t _mockHeaderBytes = 0; // byte count of the GCSV header written in begin()
#endif
};

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
extern const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount];

#endif // GYROLOGWRITER_H
