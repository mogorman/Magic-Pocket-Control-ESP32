#ifndef GYROLOGWRITER_H
#define GYROLOGWRITER_H

#include <Arduino.h>
#include <string>
#include <cstdint>

// A minimal GyroLogWriter stub so the Gyro Log screen compiles and runs
// before the full 1 kHz sampling / SD-writing logic is ported in.
//
// It exposes the same public surface the UI and main loop use:
//   - orientation (calibration) index + token table
//   - a Summary of the last clip (always invalid in this stub)
//   - SD-card status (always "no card" in this stub)
//   - isRecording() (always false in this stub)
//
// The real implementation (MPU6886 1 kHz sampling, SdFat GCSV writer,
// ring buffer, writer task) is ported in a later step.
class GyroLogWriter
{
public:
    // The 24 possible orientation tokens: 6 axis permutations (X/Y/Z) x 4
    // sign combinations. Index 0..23 maps to a token via
    // GYROLOG_ORIENTATION_TOKENS.
    static const int kOrientationCount = 24;

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

    // Begin a new log. Stub: does nothing, returns false.
    bool begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo = "");

    // Poll the IMU. Stub: does nothing.
    void poll();

    // Finalise the current log. Stub: does nothing, returns false.
    bool end();

    // Set the timecode string to record as the "end" timecode in the summary.
    void setTimecodeAtEnd(const std::string& timecode) { _summary.timecodeAtEnd = timecode; }

    // Apply a real clip name to a finalised log. Stub: does nothing.
    void applySlateName(const std::string& slateName, const std::string& extension);

    bool isRecording() const { return false; }

    const Summary& getSummary() const { return _summary; }

    // SD card status, for the on-screen diagnostic. Stub: always not ready.
    bool sdReady() const { return false; }
    const std::string& sdStatusMessage() const { return _sdStatusMessage; }

    // ---- Orientation (calibration) ----
    int getOrientationIndex() const { return _orientationIndex; }
    void setOrientationIndex(int index);
    static const char* orientationToken(int index);
    void loadOrientation();

private:
    Summary _summary;
    std::string _sdStatusMessage = "no card (stub)";
    int _orientationIndex = 0;
};

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
extern const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount];

#endif // GYROLOGWRITER_H
