#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>
#include "Arduino_DebugUtils.h" // DEBUG_INFO / DEBUG_ERROR
#include "SdFat.h" // Adafruit SdFat (SdFat, FatFile, SdSpiConfig)

// SD-backed gyro log writer.
//
// This is the DATA side only: it mounts the Core2's microSD card via SdFat
// and, for each recorded clip, writes a small text file next to where the
// GCSV data will eventually go. The 1 kHz IMU sampling / GCSV sample writing
// is NOT implemented yet (that is a later step); for now the file just proves
// the SD write path works end to end:
//
//   * begin(clipName, ...)  -> on record start: create "/<clipName>.txt" and
//                              write "hello world" plus the clip name.
//   * end()                 -> on record stop:  close the file and force the
//                              FAT directory entry to the card.
//   * applySlateName(name)  -> after the real clip name is learned from the
//                              camera (via playback), rename the file to
//                              "/<realName>.txt" and rewrite its contents to
//                              match the real name.
//
// The SD card shares the VSPI bus with the M5GFX display, so SdFat is told
// not to re-initialise the bus (USER_SPI_BEGIN) and only toggles the SD's CS
// pin (GPIO4). See ensureSd().
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
    };

    // Begin a new log: mount the SD card and create "/<clipName>.txt" with a
    // "hello world" payload. Returns true on success.
    //   clipName  : base name without extension (e.g. "clip_0001")
    //   extension : the video extension (e.g. "braw"); recorded for the summary
    //   timecode  : the camera timecode at the moment recording started
    //   lensInfo  : the lens the camera reports; may be empty
    bool begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo = "");

    // Poll the IMU. No-op for now (no sampling yet).
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

    State _state = State::Idle;
    Summary _summary;

    SdFat _sd;
    FatFile _file;
    bool _sdReady = false;
    std::string _sdStatusMessage = "not mounted yet";

    // The clip name the log was started with, and the path of the open file.
    std::string _startedName;
    std::string _extension;
    std::string _gcsvPath;
    uint64_t _finalFileSize = 0;

    int _orientationIndex = 0;
};

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
extern const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount];

#endif // GYROLOGWRITER_H
