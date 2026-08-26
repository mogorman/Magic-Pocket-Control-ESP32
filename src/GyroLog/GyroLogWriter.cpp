// [DIAG] Isolation toggle. When set, the GCSV writer skips ALL SD card I/O
// (open/write/close/sync) but still samples the MPU6886 at 1 kHz into the
// in-RAM ring. If the clip-end heap corruption disappears with this on, the
// SD/FatFs write path is the corruptor; if it persists, the IMU I2C read is.
#ifndef GYROLOG_DIAG_NO_SD_WRITE
#define GYROLOG_DIAG_NO_SD_WRITE 0
#endif

// [DIAG] Isolation toggle. When set, the SD card is MOUNTED (SD.begin) but no
// file is opened or written during the clip -- only the 1 kHz IMU sampling
// runs. If the heap stays clean, the corruption is from the file open/write
// (not the mount); if it corrupts, merely mounting the SD + 1 kHz sampling is
// the trigger.
#ifndef GYROLOG_DIAG_MOUNT_ONLY
#define GYROLOG_DIAG_MOUNT_ONLY 1
#endif

#include "GyroLogWriter.h"
#include <SD.h>
#include <M5Unified.h>
#include <nvs.h>
#include <esp_heap_caps.h> // heap_caps_free (the correct free for ps_malloc'd PSRAM)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // uxTaskGetStackHighWaterMark (stack-overflow diagnostic)
#include <math.h>
#include <cstring>
#include <time.h>
#include "Arduino_DebugUtils.h"

// FatFs, for setting a file's modification time (f_utime). The SD library is
// built on FatFs, so ff.h is on the include path.
extern "C" {
#include "ff.h"
}

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

// The year assumed for a clip's date until the real date is learned from the
// clip's file name. The camera's timecode gives a time-of-day but no date, and
// the file name only carries MMDDHHMM (no year), so a fixed default year is
// used for the initial GCSV timestamp and is corrected once the name is known.
static const int kDefaultYear = 2026;

// Convert a camera timecode string ("HH:MM:SS:FF" or drop-frame "HH:MM:SS;FF")
// to whole seconds since midnight. Returns -1 if the string can't be parsed.
// The frame field is ignored (the GCSV "timestamp" is a whole-second value and
// Gyroflow treats it as optional/informational).
static long timecodeToSeconds(const std::string& tc)
{
    if(tc.size() < 8)
        return -1;

    // Normalise the drop-frame separator (';') to ':' so one sscanf handles
    // both forms.
    std::string norm = tc;
    for(char& c : norm)
        if(c == ';')
            c = ':';

    int h, m, s, f;
    if(sscanf(norm.c_str(), "%d:%d:%d:%d", &h, &m, &s, &f) != 4)
        return -1;

    if(h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59)
        return -1;

    return (long)h * 3600 + (long)m * 60 + s;
}

// Parse the date embedded in a BMD clip file name. The camera names clips like
// "A001_08260603_C011", where the second field is MMDDHHMM (month, day, hour,
// minute) -- e.g. 08260603 = Aug 26, 06:03. The year is not in the name, so the
// caller supplies it (2026 by default). On success it fills *year, *month,
// *day, *hour, *minute and returns true.
static bool parseClipNameDate(const std::string& name, int* year, int* month, int* day, int* hour, int* minute)
{
    // Find the first underscore; the date is the field that follows it.
    size_t u = name.find('_');
    if(u == std::string::npos)
        return false;
    size_t fieldStart = u + 1;
    size_t fieldEnd = fieldStart;
    while(fieldEnd < name.size() && name[fieldEnd] != '_')
        fieldEnd++;

    std::string field = name.substr(fieldStart, fieldEnd - fieldStart);
    if(field.size() < 8)
        return false;

    int mm, dd, hh, mi;
    if(sscanf(field.c_str(), "%2d%2d%2d%2d", &mm, &dd, &hh, &mi) != 4)
        return false;

    if(mm < 1 || mm > 12 || dd < 1 || dd > 31 || hh > 23 || mi > 59)
        return false;

    *year = 0; // caller overwrites
    *month = mm;
    *day = dd;
    *hour = hh;
    *minute = mi;
    return true;
}

// Sync the ESP32 system clock (and the M5 RTC) from the camera's timecode.
//
// The camera's timecode gives a reliable time-of-day (HH:MM:SS) but no date. We
// map it onto the date we are told to use (the clip's date, or a default year
// when the date is not yet known). The result is a real UNIX timestamp, which is
// what the GCSV "timestamp" field needs (Gyroflow showed "26 years ago" when we
// wrote a seconds-since-midnight value).
//
// We set both the M5 RTC and the ESP32 system time so that time() (used to build
// the GCSV timestamp) and the FAT file timestamps reflect the value we set.
// Returns true on success.
bool GyroLogWriter::syncRtcFromTimecode(const std::string& timecode, int year)
{
    int h, m, s, f;
    std::string norm = timecode;
    for(char& c : norm)
        if(c == ';')
            c = ':';
    if(norm.size() < 8 || sscanf(norm.c_str(), "%d:%d:%d:%d", &h, &m, &s, &f) != 4)
        return false;
    if(h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59)
        return false;

    // Base date: the M5 RTC (it keeps running on the coin-cell-backed RTC8563).
    // We only borrow its month/day as a fallback; the year is the one we were
    // told to use, and the time-of-day comes from the camera's timecode.
    time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);

    // Overlay the camera's time-of-day and the requested year.
    tmv.tm_year = year - 1900;
    tmv.tm_hour = h;
    tmv.tm_min = m;
    tmv.tm_sec = s;
    tmv.tm_isdst = -1;

    // Push it to the M5 RTC and the ESP32 system clock.
    M5.Rtc.setDateTime(&tmv);
    M5.Rtc.setSystemTimeFromRtc();

    // Re-read the system time so time() reflects the value we just set.
    now = time(nullptr);
    localtime_r(&now, &tmv);
    return true;
}

// Turn a raw clip/slate name into a safe file-name base. Placeholder slate
// names the camera sends (e.g. "Next Clip") are dropped (returns empty), and
// any characters that are unsafe in a file name are replaced with '_'.
static std::string sanitiseClipName(const std::string& name)
{
    // Trim surrounding whitespace.
    size_t b = name.find_first_not_of(" \t");
    size_t e = name.find_last_not_of(" \t");
    if(b == std::string::npos)
        return ""; // blank
    std::string trimmed = name.substr(b, e - b + 1);

    // The camera uses "Next Clip" as a placeholder slate name (it has no real
    // clip name yet). Don't use that as a file name.
    if(trimmed == "Next Clip" || trimmed == "next clip")
        return "";

    std::string out;
    out.reserve(trimmed.size());
    for(char c : trimmed)
    {
        if((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
            || c == '_' || c == '-' || c == '.')
            out += c;
        else
            out += '_';
    }
    return out;
}

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
    // Read the MPU6886 directly over the internal I2C bus. A single 14-byte
    // burst from register 0x3B (ACCEL_XOUT_H) through 0x48 (GYRO_ZOUT_L)
    // returns accel (6 bytes), temperature (2 bytes, skipped), and gyro
    // (6 bytes). This is far cheaper than M5Unified's getGyro()+getAccel()
    // pair, which each does a 15-byte read and is throttled to ~10 Hz by a
    // 256 us gate. Reading directly lets us keep up with the sensor's true
    // 1 kHz ODR.
    //
    // The MPU6886 is configured by M5Unified (begin()) to:
    //   GYRO_CONFIG = 0x18  -> +/-2000 dps  -> 2000/32768 dps per LSB
    //   ACCEL_CONFIG = 0x10 -> +/-8 g       -> 8/32768 g per LSB
    //   CONFIG = 0x01        -> 44 Hz DLPF
    // and setup() then bumps SMPLRT_DIV from M5Unified's 0x03 (500 Hz) to
    // 0x00 (no divider) for a 1 kHz ODR. So the raw 16-bit values scale
    // exactly as kGscale/kAscale below.
    uint8_t buf[14];
    if(!M5.In_I2C.readRegister(kImuAddr, 0x3B, buf, 14, 400000))
        return false;

    // accel: buf[0..5]  (XH,XL,YH,YL,ZH,ZL)
    int16_t rawAx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t rawAy = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rawAz = (int16_t)((buf[4] << 8) | buf[5]);
    // gyro:  buf[8..13] (temp is buf[6..7])
    int16_t rawGx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t rawGy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t rawGz = (int16_t)((buf[12] << 8) | buf[13]);

    *ax = rawAx * kAscale;
    *ay = rawAy * kAscale;
    *az = rawAz * kAscale;
    *gx = rawGx * kGscale;
    *gy = rawGy * kGscale;
    *gz = rawGz * kGscale;
    return true;
}

bool GyroLogWriter::ensureSd()
{
#if GYROLOG_DIAG_NO_SD_WRITE
    // [DIAG] No SD I/O: pretend the card is ready so begin() proceeds and the
    // 1 kHz sampling runs without any SD mount/write.
    _sdReady = true;
    _sdStatusMessage = "diag: no SD";
    return true;
#else
    // If the filesystem is already mounted, it's ready.
    if(SD.cardSize() > 0)
    {
        _sdReady = true;
        _sdStatusMessage = "ready";
        return true;
    }

    // Not mounted yet. The Core2's microSD slot is on the SPI bus with its
    // chip-select on GPIO4. The Arduino SD library's default CS is the
    // variant's SS pin (GPIO5 on the Core2), which is actually the LCD's CS,
    // so we must pass the correct pin explicitly or the card never mounts.
    if(!SD.begin(4))
    {
        _sdReady = false;
        _sdStatusMessage = "mount failed (no card / not FAT?)";
        return false;
    }

    // begin() returned true, but double-check a card is actually present.
    if(SD.cardSize() > 0)
    {
        _sdReady = true;
        _sdStatusMessage = "ready";
        return true;
    }

    _sdReady = false;
    _sdStatusMessage = "no card detected";
    return false;
#endif
}

// Force the FAT volume's cached directory entries (file sizes + cluster
// pointers) to be written to the card. File::close() only flushes the file's
// own data buffer; the directory entry that records a file's size lives in a
// separate RAM cache that FatFs otherwise only commits on the next directory
// operation or on unmount. Because we close the log and then just sit idle,
// that directory update never reaches the card, so the file shows as 0 bytes
// when the card is read on a computer. Unmounting and remounting forces the
// commit. The Core2 mounts the microSD on CS pin 4 (see ensureSd()).
void GyroLogWriter::syncVolume()
{
    if(SD.cardSize() == 0)
        return; // not mounted

    // [DIAG] Verify heap integrity right before the unmount/remount. The crash
    // has been in the SD.begin() -> ff_memalloc here; if the heap is already
    // corrupt before we get here, this prints the offending block so we know
    // the corruption happened earlier (during the clip) rather than in the
    // remount itself.
    if(!heap_caps_check_integrity_all(true))
        DEBUG_ERROR("[GYRO-DIAG] syncVolume(): heap ALREADY corrupt before SD remount (internal free=%lu, psram free=%lu)",
            (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());

    SD.end();
    SD.begin(4);
}

bool GyroLogWriter::begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo)
{
    if(_state == State::Recording)
        return false; // already recording

    loadOrientation();

    // Make sure the SD card is mounted before we try to open the file.
    if(!ensureSd())
    {
        DEBUG_INFO("[GYRO-DIAG] begin(): ensureSd FAILED (msg='%s')", _sdStatusMessage.c_str());
        return false;
    }
    DEBUG_INFO("[GYRO-DIAG] begin(): SD ready, cardSize=%lu", (unsigned long)SD.cardSize());

    // Sanitise the clip name (drops placeholder slate names like "Next Clip"
    // and replaces any file-name-unsafe characters). The caller falls back to a
    // generated "clip_NNNN" name when this comes back empty.
    _startedName = sanitiseClipName(clipName);
    if(_startedName.empty())
        _startedName = clipName; // shouldn't happen; caller handles the fallback
    _extension = extension;
    _videoFileName = _startedName + "." + extension;
    _lensInfo = lensInfo;

    char path[128];
    snprintf(path, sizeof(path), "/%s.gcsv", _startedName.c_str());
    _gcsvPath = path;
#if !GYROLOG_DIAG_NO_SD_WRITE && !GYROLOG_DIAG_MOUNT_ONLY
    _file = SD.open(path, FILE_WRITE);
    if(!_file)
        return false;
#endif

    // The GCSV "timestamp" field is a UNIX timestamp (seconds since 1970-01-01
    // 00:00:00 UTC) and is optional (Gyroflow syncs on the "t" column, not
    // this). We sync the ESP32 clock from the camera's timecode. The clip's real
    // date is not known yet (it only arrives after the clip, via the file name),
    // so we use a default year (2026) for now; applySlateName() corrects this to
    // the real date once the name is known.
    long timestampEpoch = 0;
    if(syncRtcFromTimecode(timecode, kDefaultYear))
        timestampEpoch = (long)time(nullptr);
    _timestampEpoch = timestampEpoch;
    // The GCSV "lens_info" field is free-form extra lens/FOV metadata. We use
    // it to record the lens the camera reports (cat=12 param=9, e.g.
    // "Canon EF-S 18-55mm f/3.5-5.6 IS II"). Only written when known.
    char header[700];
    int n = snprintf(header, sizeof(header),
        "GYROFLOW IMU LOG\n"
        "version,1.3\n"
        "id,m5stack-core2-mpu6886\n"
        "orientation,%s\n"
        "note,M5Stack Core2 gyro log ~1kHz; start TC %s\n"
        "fwversion,1.0.0\n"
        "timestamp,%ld\n"
        "vendor,m5stack\n"
        "videofilename,%s\n",
        GyroLogWriter::orientationToken(_orientationIndex),
        timecode.c_str(),
        timestampEpoch,
        _videoFileName.c_str());
    if(!_lensInfo.empty())
    {
        // The lens string is free text from the camera; strip any commas so it
        // can't break the CSV field, then append it as its own line.
        std::string lens = _lensInfo;
        for(char& c : lens)
            if(c == ',' || c == '\n' || c == '\r')
                c = ' ';
        n += snprintf(header + n, sizeof(header) - (size_t)n,
            "lens_info,%s\n", lens.c_str());
    }
    n += snprintf(header + n, sizeof(header) - (size_t)n,
        "tscale,%s\n"
        "gscale,%.11f\n"
        "ascale,%.11f\n"
        "t,gx,gy,gz,ax,ay,az\n",
        kTscale,
        kGscale,
        kAscale);
#if !GYROLOG_DIAG_NO_SD_WRITE && !GYROLOG_DIAG_MOUNT_ONLY
    _file.write((const uint8_t*)header, n);
#endif

    if(!allocRing())
    {
#if !GYROLOG_DIAG_NO_SD_WRITE && !GYROLOG_DIAG_MOUNT_ONLY
        _file.close();
#endif
        return false;
    }

    _tMs = 0;
    _startMicros = micros();
    _lastSampleMicros = _startMicros;
    _lastDrainMicros = _startMicros;

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

    // [DIAG] Once per second while recording, verify the internal + PSRAM heap
    // integrity. If the 1 kHz sampling/SD-write path is corrupting the heap,
    // this catches it *during* the clip and prints the exact corrupt block,
    // instead of only surfacing at the clip-end remount. multi_heap_check
    // prints the offending address/capabilities on failure.
    static uint32_t lastHeapCheck = 0;
    if(now - lastHeapCheck >= 1000000)
    {
        lastHeapCheck = now;
        bool ok = heap_caps_check_integrity_all(true);
        if(!ok)
            DEBUG_ERROR("[GYRO-DIAG] poll(): HEAP INTEGRITY FAIL at t=%lu ms (internal free=%lu, psram free=%lu)",
                (unsigned long)_tMs, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
    }

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

    // t is the real elapsed time in ms since the log started (micros-based),
    // so it stays correct even if a sample is ever dropped.
    _tMs = (uint32_t)((now - _startMicros) / 1000);

    char row[64];
    int n = snprintf(row, sizeof(row), "%lu,%ld,%ld,%ld,%ld,%ld,%ld\n",
        (unsigned long)_tMs, igx, igy, igz, iax, iay, iaz);

    // Append to the ring buffer if there is room; otherwise drop (we never
    // block the 1 kHz cadence on a full buffer).
    if(_ringCount >= kRingSize)
        return;
    if(_ringCount + (size_t)n > kRingSize)
        return;

    memcpy(_ring + _ringWrite, row, (size_t)n);
    _ringWrite = (_ringWrite + n) % kRingSize;
    _ringCount += (size_t)n;

    // Drain to the file, but rate-limit the SD writes. The 1 kHz sampling fills
    // the ring continuously; we flush it to the SD card at most once per 50 ms
    // (20 Hz) and only once a decent chunk has accumulated. This keeps the SD /
    // SPI / DMA activity far below the sample rate, which is the suspected cause
    // of the internal-heap corruption. At 1 kHz with ~30-byte rows, 50 ms is
    // ~1.5 KB, well within the 64 KB ring, so nothing is dropped.
    if(now - _lastDrainMicros >= 50000 && _ringCount >= 1024)
    {
        _lastDrainMicros = now;
        drainRing();
    }
}

void GyroLogWriter::drainRing()
{
    if(_ringCount == 0)
        return;

#if GYROLOG_DIAG_NO_SD_WRITE
    // [DIAG] No SD: just discard the buffered rows so the ring keeps turning.
    // This lets us test whether the 1 kHz IMU sampling alone (no SD writes)
    // corrupts the heap.
    _ringRead = 0;
    _ringWrite = 0;
    _ringCount = 0;
    return;
#else
    if(!_file)
        return;

    // Copy out the contiguous available bytes and write them. (No per-drain
    // flush(): data is committed by the flush()/close in end().)
    size_t head = (kRingSize - _ringRead) < _ringCount ? (kRingSize - _ringRead) : _ringCount;
    size_t total = _ringCount;

    if(head)
        _file.write((const uint8_t*)(_ring + _ringRead), head);
    if(total > head)
        _file.write((const uint8_t*)(_ring + 0), total - head);

    _ringRead = (_ringRead + total) % kRingSize;
    _ringCount = 0;
#endif
}

// Rewrite the "videofilename" header line of a *closed* .gcsv file in place.
//
// The file was written with a generic name (e.g. "clip_0001.mov") and later
// renamed to the real clip name. The "videofilename" field inside the header
// still holds the old name, so we read the whole file, replace that one line,
// and write it back. The file is small (a few hundred KB at most) and this runs
// once per clip, so a full read/rewrite is fine. Returns true on success.
bool GyroLogWriter::rewriteVideoFileName(const std::string& path, const std::string& newVideoFileName)
{
    File f = SD.open(path.c_str(), FILE_READ);
    if(!f)
        return false;

    size_t size = f.size();
    std::string content;
    content.reserve(size + 64);
    int c;
    while((c = f.read()) != -1)
        content += (char)c;
    f.close();

    // Locate the "videofilename," line and replace everything up to its end of
    // line with the new value.
    const char* kKey = "videofilename,";
    size_t keyPos = content.find(kKey);
    if(keyPos == std::string::npos)
        return false; // no such field; leave the file alone
    size_t lineEnd = content.find('\n', keyPos);
    if(lineEnd == std::string::npos)
        lineEnd = content.size();

    std::string out;
    out.reserve(content.size() + 64);
    out.append(content, 0, keyPos + strlen(kKey));
    out += newVideoFileName;
    if(lineEnd < content.size())
        out.append(content, lineEnd, content.size() - lineEnd); // keep the trailing \n and the rest

    // Write it back.
    File w = SD.open(path.c_str(), FILE_WRITE);
    if(!w)
        return false;
    w.write((const uint8_t*)out.data(), out.size());
    w.close();
    return true;
}

// Rewrite the "timestamp" header line of a closed .gcsv file at `path` to the
// given UNIX epoch. Same read-modify-write approach as rewriteVideoFileName().
bool GyroLogWriter::rewriteTimestamp(const std::string& path, long epoch)
{
    File f = SD.open(path.c_str(), FILE_READ);
    if(!f)
        return false;

    size_t size = f.size();
    std::string content;
    content.reserve(size + 64);
    int c;
    while((c = f.read()) != -1)
        content += (char)c;
    f.close();

    const char* kKey = "timestamp,";
    size_t keyPos = content.find(kKey);
    if(keyPos == std::string::npos)
        return false;
    size_t lineEnd = content.find('\n', keyPos);
    if(lineEnd == std::string::npos)
        lineEnd = content.size();

    char val[24];
    int vn = snprintf(val, sizeof(val), "%ld", epoch);

    std::string out;
    out.reserve(content.size() + 64);
    out.append(content, 0, keyPos + strlen(kKey));
    out.append(val, vn);
    if(lineEnd < content.size())
        out.append(content, lineEnd, content.size() - lineEnd);

    File w = SD.open(path.c_str(), FILE_WRITE);
    if(!w)
        return false;
    w.write((const uint8_t*)out.data(), out.size());
    w.close();
    return true;
}

// Set the FAT modification time of the file at `path` to `epoch` (UNIX
// seconds) using FatFs's f_utime(). The SD card is mounted at "/sd" (see
// SD.h), so the FatFs path is "/sd" + the path we open the file with. The
// caller sets the system clock to the clip's date first, so the timestamp we
// pass matches it. syncVolume() then commits the directory entry to the card.
bool GyroLogWriter::setFileMtime(const std::string& path, long epoch)
{
    char fatfsPath[160];
    snprintf(fatfsPath, sizeof(fatfsPath), "/sd%s", path.c_str());

    // Convert the UNIX epoch to the packed FAT date/time words.
    time_t t = (time_t)epoch;
    struct tm tmv;
    localtime_r(&t, &tmv);
    uint16_t fdate = (uint16_t)(((tmv.tm_year + 1980) << 9) | ((tmv.tm_mon + 1) << 5) | tmv.tm_mday);
    uint16_t ftime = (uint16_t)((tmv.tm_hour << 11) | (tmv.tm_min << 5) | (tmv.tm_sec / 2));

    FILINFO fi;
    memset(&fi, 0, sizeof(fi));
    fi.fdate = fdate;
    fi.ftime = ftime;
    return f_utime(fatfsPath, &fi) == FR_OK;
}

void GyroLogWriter::applySlateName(const std::string& slateName, const std::string& extension)
{
    // Only meaningful right after a log has been finalised.
    if(_state != State::Idle)
        return;

    // Sanitise the slate name; a placeholder ("Next Clip") or blank name comes
    // back empty, in which case we keep the name we started with.
    std::string name = sanitiseClipName(slateName);
    if(name.empty())
        return;
    if(name == _startedName)
        return; // already using this name

    char oldPath[128];
    char newPath[128];
    snprintf(oldPath, sizeof(oldPath), "/%s.gcsv", _startedName.c_str());
    snprintf(newPath, sizeof(newPath), "/%s.gcsv", name.c_str());

    // The file was already closed (and its directory entry committed to the
    // card) by end(). Move it to the real clip name with SD.rename() (a FatFs
    // f_rename).
    if(std::strcmp(oldPath, newPath) != 0)
    {
        bool renamed = SD.rename(oldPath, newPath);
        DEBUG_INFO("[GYRO-RENAME] rename '%s' -> '%s' ok=%d", oldPath, newPath, (int)renamed);
        // The rename dirties the directory again; commit it to the card.
        syncVolume();
    }

    // Update the "videofilename" field inside the (now renamed) file so it
    // matches the real video file name, not the generic name we started with.
    std::string newVideoFileName = name + "." + extension;
    if(rewriteVideoFileName(newPath, newVideoFileName))
        DEBUG_INFO("[GYRO-RENAME] updated videofilename to '%s'", newVideoFileName.c_str());
    else
        DEBUG_INFO("[GYRO-RENAME] could not update videofilename in '%s'", newPath);

    // Now that we know the real clip name, correct the date. The name carries
    // MMDDHHMM (e.g. 08260603 = Aug 26, 06:03) but no year, so we use the
    // default year. We re-sync the clock to that date, rewrite the GCSV
    // "timestamp" field, and touch the file so its FAT modification time matches.
    int year, month, day, hour, minute;
    if(parseClipNameDate(name, &year, &month, &day, &hour, &minute))
    {
        year = kDefaultYear;
        struct tm tmv;
        tmv.tm_year = year - 1900;
        tmv.tm_mon = month - 1;
        tmv.tm_mday = day;
        tmv.tm_hour = hour;
        tmv.tm_min = minute;
        tmv.tm_sec = 0;
        tmv.tm_isdst = -1;
        long correctedEpoch = (long)mktime(&tmv);

        // Set the system clock + M5 RTC to the clip's date (so the touch below
        // stamps the right time) and record the corrected epoch.
        char tc[16];
        snprintf(tc, sizeof(tc), "%02d:%02d:%02d:00", hour, minute, 0);
        syncRtcFromTimecode(tc, year);
        _timestampEpoch = correctedEpoch;

        if(rewriteTimestamp(newPath, correctedEpoch))
            DEBUG_INFO("[GYRO-RENAME] corrected timestamp to %ld (%04d-%02d-%02d %02d:%02d)",
              correctedEpoch, year, month, day, hour, minute);
        if(setFileMtime(newPath, correctedEpoch))
            DEBUG_INFO("[GYRO-RENAME] set file mtime to %ld", correctedEpoch);
    }

    // The rewrite/touch dirty the directory again; commit it to the card.
    syncVolume();

    // Reflect the new names in the summary shown on the Gyro Log screen.
    _startedName = name;
    _extension = extension;
    _videoFileName = newVideoFileName;
    _summary.fileName = name + ".gcsv";
    _summary.videoFileName = _videoFileName;
}

bool GyroLogWriter::end()
{
    if(_state != State::Recording)
        return false;

    // [DIAG] One-shot health check right before the SD remount that has been
    // crashing (heap TLSF assert in ff_memalloc). end() runs on the main loop
    // task, so uxTaskGetStackHighWaterMark(NULL) reports that task's stack
    // high-water mark (bytes still free at its deepest point). A value near 0
    // means the loop stack is overflowing, which would corrupt the adjacent
    // internal heap. Also reports free internal/PSRAM heap.
    {
        UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
        DEBUG_INFO("[GYRO-DIAG] end(): loop stack HWM=%u bytes free, internal free=%lu, psram free=%lu",
            (unsigned)hw, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
    }

    // Flush any remaining buffered samples.
    drainRing();
#if !GYROLOG_DIAG_NO_SD_WRITE && !GYROLOG_DIAG_MOUNT_ONLY
    _file.flush();

    // Capture the summary before closing.
    _summary.valid = true;
    _summary.fileName = _startedName + ".gcsv";
    _summary.videoFileName = _videoFileName;
    _summary.durationMs = _tMs;
    _summary.fileSizeBytes = _file.size();
    _summary.totalBytes = SD.totalBytes();
    _summary.freeBytes = SD.totalBytes() - SD.usedBytes();

    _file.close();

    // Commit the file's directory entry (size + clusters) to the card so the
    // file is not read back as 0 bytes on a computer.
    syncVolume();
#else
    // [DIAG] No SD: no file to close/sync. Just record a minimal summary.
    _summary.valid = true;
    _summary.fileName = _startedName + ".gcsv";
    _summary.videoFileName = _videoFileName;
    _summary.durationMs = _tMs;
#endif

    if(_ring)
    {
        // _ring was allocated with ps_malloc (the SPIRAM/PSRAM heap), so it
        // must be freed with heap_caps_free, NOT vPortFree. vPortFree frees from
        // the internal DRAM heap; freeing a PSRAM pointer with it corrupts the
        // internal heap's TLSF free list (assert "block must be free" / "free
        // list cannot have a null entry" on the next malloc).
        heap_caps_free(_ring);
        _ring = nullptr;
    }

    _state = State::Idle;
    return true;
}
