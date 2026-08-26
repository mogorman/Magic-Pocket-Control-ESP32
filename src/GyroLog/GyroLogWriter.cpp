#include "GyroLogWriter.h"
#include <SD.h>
#include <M5Unified.h>
#include <nvs.h>
#include <math.h>

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
//
// A GCSV orientation token is a 3-character string. Each character is one of
// the sensor axes (X, Y, Z) and the characters are listed in the order they
// map to the GCSV gx / gy / gz columns. A lower-case character means that
// axis is inverted (e.g. "xYz" = gx from -X, gy from +Y, gz from -Z).
//
// This table is the full set of 24 right-handed orientation tokens (6 axis
// permutations x 4 sign combinations), in a fixed order so the A/B buttons can
// step through them deterministically. The index is what is persisted in NVS.
const char* const GYROLOG_ORIENTATION_TOKENS[GyroLogWriter::kOrientationCount] =
{
    // Permutation X Y Z
    "XYZ", "XyZ", "XYz", "xYz",
    // Permutation X Z Y
    "XZY", "XzY", "XZy", "xZy",
    // Permutation Y X Z
    "YXZ", "YxZ", "YXz", "yXz",
    // Permutation Y Z X
    "YZX", "YzX", "YZx", "yZx",
    // Permutation Z X Y
    "ZXY", "ZxY", "ZXy", "zXy",
    // Permutation Z Y X
    "ZYX", "ZyX", "ZYx", "zYx"
};

// The orientation is calibrated empirically: the user lays the unit flat,
// reads the live gyro/accel numbers on the Gyro Log screen, and steps through
// these tokens with the A/B buttons until the token matches how the MPU6886 is
// physically mounted (screen facing up and flat). The chosen index is
// persisted in NVS. Gyroflow uses the token to rotate the data into the
// camera's frame, so the token must describe the sensor's real orientation.
//
// gscale: raw gyro (deg/s) -> rad/s. MPU6886 is configured +/-2000 deg/s.
//   gscale = (2000 * PI / 180) / 32768
static const float kGscale = (2000.0f * 3.141592653589793f / 180.0f) / 32768.0f;
// ascale: raw accel (g) -> g. MPU6886 is configured +/-8 g.
//   ascale = 8 / 32768
static const float kAscale = 8.0f / 32768.0f;

// tscale: t is in ms, so seconds = t * 0.001
static const char* kTscale = "0.001";

const char* GyroLogWriter::kNvsNamespace = "GyroLog";
const char* GyroLogWriter::kNvsKeyOrientation = "orient";

const char* GyroLogWriter::orientationToken(int index)
{
    if(index < 0) index = 0;
    if(index >= kOrientationCount) index = kOrientationCount - 1;
    return GYROLOG_ORIENTATION_TOKENS[index];
}

bool GyroLogWriter::allocRing()
{
    _ring = (char*)ps_malloc(kRingSize);
    if(!_ring)
        return false;
    _ringWrite = 0;
    _ringRead = 0;
    _ringCount = 0;
    return true;
}

void GyroLogWriter::loadOrientation()
{
    nvs_handle_t handle = 0;
    if(nvs_open(GyroLogWriter::kNvsNamespace, NVS_READONLY, &handle) == ESP_OK)
    {
        uint32_t val = 0;
        if(nvs_get_u32(handle, GyroLogWriter::kNvsKeyOrientation, &val) == ESP_OK)
            _orientationIndex = (val >= kOrientationCount) ? 0 : (int)val;
        nvs_close(handle);
    }
    // Default stays 0 if nothing stored.
}

void GyroLogWriter::saveOrientation()
{
    nvs_handle_t handle = 0;
    if(nvs_open(GyroLogWriter::kNvsNamespace, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_u32(handle, GyroLogWriter::kNvsKeyOrientation, (uint32_t)_orientationIndex);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void GyroLogWriter::setOrientationIndex(int index)
{
    if(index < 0) index = 0;
    if(index >= kOrientationCount) index = kOrientationCount - 1;
    _orientationIndex = index;
    saveOrientation();
}

bool GyroLogWriter::readImu(float* gx, float* gy, float* gz, float* ax, float* ay, float* az)
{
    // M5.Imu returns gyro in rad/s and accel in g (see M5Unified IMU_Class).
    if(!M5.Imu.isEnabled())
        return false;
    bool okG = M5.Imu.getGyro(gx, gy, gz);
    bool okA = M5.Imu.getAccel(ax, ay, az);
    return okG && okA;
}

bool GyroLogWriter::begin(const std::string& clipName, const std::string& extension, const std::string& timecode)
{
    if(_state == State::Recording)
        return false; // already recording

    loadOrientation();

    // Make sure the SD card is up.
    if(!SD.cardSize())
    {
        if(!SD.begin())
            return false;
    }

    _startedName = clipName;
    _extension = extension;
    _renamedFromSlate = false;

    char path[128];
    snprintf(path, sizeof(path), "/%s.gcsv", clipName.c_str());
    _file = SD.open(path, FILE_WRITE);
    if(!_file)
        return false;

    // The GCSV "timestamp" field is documented as a UNIX timestamp and is
    // optional (Gyroflow syncs on the "t" column, not this). We leave it 0 and
    // record the camera's timecode at the moment recording started in "note",
    // which is more useful for matching the log to the clip.
    char header[600];
    int n = snprintf(header, sizeof(header),
        "GYROFLOW IMU LOG\n"
        "version,1.3\n"
        "id,m5stack-core2-mpu6886\n"
        "orientation,%s\n"
        "note,M5Stack Core2 gyro log ~1kHz; start TC %s\n"
        "fwversion,1.0.0\n"
        "timestamp,0\n"
        "vendor,m5stack\n"
        "videofilename,%s.%s\n"
        "tscale,%s\n"
        "gscale,%.11f\n"
        "ascale,%.11f\n"
        "t,gx,gy,gz,ax,ay,az\n",
        GyroLogWriter::orientationToken(_orientationIndex),
        timecode.c_str(),
        clipName.c_str(),
        extension.c_str(),
        kTscale,
        kGscale,
        kAscale);
    _file.write((const uint8_t*)header, n);

    if(!allocRing())
    {
        _file.close();
        return false;
    }

    _tMs = 0;
    _lastSampleMicros = micros();

    _state = State::Recording;
    return true;
}

void GyroLogWriter::poll()
{
    if(_state != State::Recording)
        return;

    // Sample at ~1 kHz: only take a new sample if >= 1 ms since the last.
    uint32_t now = micros();
    if(now - _lastSampleMicros < 1000)
        return;
    _lastSampleMicros = now;

    float gx, gy, gz, ax, ay, az;
    if(!readImu(&gx, &gy, &gz, &ax, &ay, &az))
        return;

    // Quantise to the fixed-point scale Gyroflow expects.
    long igx = lroundf(gx / kGscale);
    long igy = lroundf(gy / kGscale);
    long igz = lroundf(gz / kGscale);
    long iax = lroundf(ax / kAscale);
    long iay = lroundf(ay / kAscale);
    long iaz = lroundf(az / kAscale);

    char row[64];
    int n = snprintf(row, sizeof(row), "%lu,%ld,%ld,%ld,%ld,%ld,%ld\n",
        (unsigned long)_tMs, igx, igy, igz, iax, iay, iaz);
    _tMs += 1;

    // Append to the ring buffer if there is room; otherwise drop (we never
    // block the 1 kHz cadence on a full buffer).
    if(_ringCount >= kRingSize)
        return;
    if(_ringCount + (size_t)n > kRingSize)
        return;

    memcpy(_ring + _ringWrite, row, (size_t)n);
    _ringWrite = (_ringWrite + n) % kRingSize;
    _ringCount += (size_t)n;

    // Drain to the file if we have a decent chunk buffered.
    if(_ringCount >= 4096)
        drainRing();
}

void GyroLogWriter::drainRing()
{
    if(_ringCount == 0 || !_file)
        return;

    // Copy out the contiguous available bytes and write them.
    size_t head = (kRingSize - _ringRead) < _ringCount ? (kRingSize - _ringRead) : _ringCount;
    size_t total = _ringCount;

    if(head)
        _file.write((const uint8_t*)(_ring + _ringRead), head);
    if(total > head)
        _file.write((const uint8_t*)(_ring + 0), total - head);

    _ringRead = (_ringRead + total) % kRingSize;
    _ringCount = 0;
    _file.flush();
}

void GyroLogWriter::maybeRenameFromSlate(const std::string& slateName, const std::string& extension)
{
    if(_state != State::Recording)
        return;
    if(_renamedFromSlate)
        return;
    if(slateName.empty())
        return;
    if(slateName == _startedName)
        return; // already using this name

    char oldPath[128];
    char newPath[128];
    snprintf(oldPath, sizeof(oldPath), "/%s.gcsv", _startedName.c_str());
    snprintf(newPath, sizeof(newPath), "/%s.gcsv", slateName.c_str());

    // Close, rename, reopen.
    _file.close();
    if(SD.rename(oldPath, newPath))
    {
        _file = SD.open(newPath, FILE_WRITE);
        if(_file)
        {
            _startedName = slateName;
            _extension = extension;
            _renamedFromSlate = true;
        }
    }
    else
    {
        // Reopen the old path so we keep logging.
        _file = SD.open(oldPath, FILE_WRITE);
    }
}

bool GyroLogWriter::end()
{
    if(_state != State::Recording)
        return false;

    // Flush any remaining buffered samples.
    drainRing();
    _file.flush();

    // Capture the summary before closing.
    _summary.valid = true;
    _summary.fileName = _startedName + ".gcsv";
    _summary.videoFileName = _startedName + "." + _extension;
    _summary.durationMs = _tMs;
    _summary.fileSizeBytes = _file.size();
    _summary.totalBytes = SD.totalBytes();
    _summary.freeBytes = SD.totalBytes() - SD.usedBytes();

    _file.close();

    if(_ring)
    {
        vPortFree(_ring);
        _ring = nullptr;
    }

    _state = State::Idle;
    return true;
}
