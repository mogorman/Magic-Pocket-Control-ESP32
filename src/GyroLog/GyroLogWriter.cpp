#include "GyroLogWriter.h"
#include <M5Unified.h>
#include <nvs.h>
#include <esp_heap_caps.h> // heap_caps_check_integrity_all (diagnostics)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // uxTaskGetStackHighWaterMark (stack-overflow diagnostic)
#include <esp_timer.h> // esp_timer_get_time() for the microsecond-accurate 1 kHz sampling grid
#include <math.h>
#include <cstring>
#include <time.h>
#include "Arduino_DebugUtils.h"

// 1 = keep the per-second sampler diagnostic (loop rate + I2C read time). 0 =
// quiet (the default) so the 1 kHz sampling path does no per-second logging.
#ifndef GYROLOG_DEBUG
#define GYROLOG_DEBUG 0
#endif
// [DIAG] 1 = the writer copies the ring out but skips the actual SD card write.
// Used to test whether the SD/SPI write path is what corrupts the heap on long
// recordings. 0 = normal (write to the card).
#ifndef GYRO_SKIP_SD_WRITE
#define GYRO_SKIP_SD_WRITE 0
#endif
// [DIAG] 1 = the sampler skips the actual MPU6886 I2C read (writes a dummy row
// instead) to test whether the I2C read is what leaks internal RAM. 0 = normal.
#ifndef GYRO_SKIP_IMU_READ
#define GYRO_SKIP_IMU_READ 0
#endif
// [DIAG] 1 = the sampler task just sleeps (no ring write, no semaphore) to test
// whether the sampler's per-tick work is what leaks internal RAM. 0 = normal.
#ifndef GYRO_SKIP_SAMPLER_WORK
#define GYRO_SKIP_SAMPLER_WORK 0
#endif
// [DIAG] 1 = the writer task just sleeps (no ring drain, no SD write) to test
// whether the writer's work is what leaks internal RAM. 0 = normal.
#ifndef GYRO_SKIP_WRITER_WORK
#define GYRO_SKIP_WRITER_WORK 0
#endif
// [DIAG] 1 = the sampler does the I2C read but skips the ring write + semaphore
// to test whether the ring/semaphore path is the leak source. 0 = normal.
#ifndef GYRO_SKIP_RING_WRITE
#define GYRO_SKIP_RING_WRITE 0
#endif

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

// Configure the sensor for OUTPUT-REGISTER polling at ~1 kHz. We wake the sensor
// (PWR_MGMT_1: clear SLEEP, select the PLL gyro-X clock for stable timing) and set
// a DLPF/SMPLRT_DIV that gives a sample rate comfortably above our 1 kHz poll rate
// (so the output registers always hold a fresh sample when we read them). We do
// NOT enable the FIFO -- we read the output registers (0x3B+) directly at 1 kHz
// from the sampler task. This keeps the I2C load trivial (~14 KB/s) and the "t"
// index dense by construction, avoiding the FIFO's 3.8 kHz rate that the I2C bus
// can't drain losslessly.
void GyroLogWriter::configurePolling()
{
    // Wake the sensor and select the PLL gyro-X clock (PWR_MGMT_1 = 0x40:
    // bit6=1 PLL gyro X, SLEEP clear). This is the standard active clock.
    M5.In_I2C.writeRegister8(kImuAddr, 0x6B, 0x40, 400000);
    vTaskDelay(pdMS_TO_TICKS(10)); // let the PLL lock
    // DLPF (CONFIG 0x1A) = 0x01 and SMPLRT_DIV (0x19) = 0. With the clone's
    // ~3.8 kHz base rate, this keeps the sensor sampling fast; we simply read the
    // latest output-register value once per 1 ms tick. (The exact sensor ODR
    // doesn't matter for a 1 kHz poll -- we just need the registers to hold a
    // fresh sample, which they do at any ODR >= 1 kHz.)
    M5.In_I2C.writeRegister8(kImuAddr, 0x19, 0x00, 400000); // SMPLRT_DIV = 0
    M5.In_I2C.writeRegister8(kImuAddr, 0x1A, 0x01, 400000); // CONFIG: DLPF = 0x01
    // Make sure the FIFO is disabled (we're not using it in this mode).
    M5.In_I2C.writeRegister8(kImuAddr, 0x6A, 0x00, 400000); // FIFO_EN = 0

    // For the 1 kHz poll mode the output rate is exactly our poll rate (1 kHz),
    // so tscale is simply 1 ms per sample. We set it here (not measured) because
    // the poll cadence -- not the sensor's internal ODR -- defines the sample
    // spacing in the file.
    _tscale = 0.001f; // 1 ms per sample (1 kHz)
    DEBUG_INFO("[GYRO] configurePolling(): 1 kHz output-register poll mode, tscale=0.001000 s");
    _fifoConfigured = true;
}

// Read the latest gyro+accel sample from the output registers and append one
// dense GCSV row. The output registers (0x3B..0x48) always hold the most recent
// sample: accel X/Y/Z (6 B), temp (2 B), gyro X/Y/Z (6 B) = 14 bytes. We read
// them in a single I2C transaction and write one row with the running _fifoSeq
// as the "t" index. Because the sampler calls this exactly once per 1 ms tick,
// the "t" index is dense (0,1,2,...) and the timeline is accurate.
uint32_t GyroLogWriter::pollOutputRegisters()
{
    uint8_t buf[14];
#if GYRO_SKIP_IMU_READ
    // [DIAG] skip the real I2C read; fill with a pattern so the loop/ring path
    // still runs. Used to test whether the I2C read is the source of the leak.
    for(int i = 0; i < 14; i++) buf[i] = (uint8_t)(i * 7 + 1);
#else
    // Read 14 bytes starting at 0x3B (the output-register block). Retry a couple
    // of times on an I2C glitch (a single atomic read; a failure means no data
    // was transferred, so a retry re-reads the same latest sample).
    bool ok = false;
    for(int attempt = 0; attempt < 3 && !ok; attempt++)
    {
        ok = M5.In_I2C.readRegister(kImuAddr, 0x3B, buf, 14, _i2cHz);
    }
    if(!ok)
    {
        _i2cFailures++;
        return 0; // this tick's sample is lost (a single gap); the next tick recovers
    }
#endif

    int16_t rawAx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t rawAy = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rawAz = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6..7] = temperature (skipped)
    int16_t rawGx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t rawGy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t rawGz = (int16_t)((buf[12] << 8) | buf[13]);

    char row[64];
    int n = snprintf(row, sizeof(row), "%lu,%ld,%ld,%ld,%ld,%ld,%ld\n",
        (unsigned long)_fifoSeq,
        (long)rawGx, (long)rawGy, (long)rawGz,
        (long)rawAx, (long)rawAy, (long)rawAz);

#if GYRO_SKIP_RING_WRITE
    // [DIAG] skip the ring write + semaphore (tests whether the ring/semaphore
    // path is the leak source; the I2C read above still runs).
    (void)n;
#else
    if(xSemaphoreTake(_ringMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        _ring.write((const uint8_t*)row, (size_t)n);
        xSemaphoreGive(_ringMutex);
        if(_dataSem)
            xSemaphoreGive(_dataSem); // wake the writer
    }
#endif
    _fifoSeq++;
    return 1;
}

GyroLogWriter::~GyroLogWriter()
{
    // Stop the writer task (if running) and delete the FreeRTOS objects.
    stopWriterTask();
    if(_ringMutex)
    {
        vSemaphoreDelete(_ringMutex);
        _ringMutex = nullptr;
    }
    if(_dataSem)
    {
        vSemaphoreDelete(_dataSem);
        _dataSem = nullptr;
    }
}

// Make sure the SD card is mounted via SdFat. The Core2's microSD slot is on
// the SPI bus with its chip-select on GPIO4. We share the SPI bus with the
// M5GFX display (SHARED_SPI), so SdFat toggles only the SD's CS pin. The SPI
// clock is capped at 40 MHz, which is comfortably within the card's spec and
// leaves headroom for the display's own SPI traffic.
bool GyroLogWriter::ensureSd()
{
    // If the volume is already mounted and a valid FAT volume is present, it's
    // ready. fatType() is non-zero only when a real FAT16/32/exFAT volume is
    // mounted, so this doubles as the "card present" check.
    if(_sd.fatType() != 0)
    {
        _sdReady = true;
        _sdStatusMessage = "ready";
        return true;
    }

    // The Core2's SD card shares the VSPI bus with the M5GFX display, which
    // initializes that bus itself (spi_bus_initialize(VSPI)). Tell SdFat NOT to
    // re-begin the bus (USER_SPI_BEGIN) -- re-initializing an already-active SPI
    // bus corrupts the MISO path and the card read times out (0xFF). Bring the
    // Arduino SPI object up first, then let SdFat only do begin/endTransaction.
    SPI.begin();
    // 20 MHz SPI for the SD card (verified to work on the Core2). A faster card
    // means each SD write is shorter, so the writer's preemptions of the 1 kHz
    // sampler are briefer and we lose fewer samples to the card writes.
    if(!_sd.begin(SdSpiConfig(4, SHARED_SPI | USER_SPI_BEGIN, SD_SCK_MHZ(20))))
    {
        _sdReady = false;
        _sdStatusMessage = "mount failed (no card / not FAT?)";
        DEBUG_INFO("[GYRO-DIAG] ensureSd: SdFat begin FAILED cardType=%d sdErrorCode=0x%02X sdErrorData=0x%08lX",
            (int)_sd.card()->type(), (unsigned)_sd.sdErrorCode(), (unsigned long)_sd.sdErrorData());
        return false;
    }

    if(_sd.fatType() != 0)
    {
        _sdReady = true;
        _sdStatusMessage = "ready";
        return true;
    }

    _sdReady = false;
    _sdStatusMessage = "no card detected";
    return false;
}

// Force the FAT volume's cached directory entries (file sizes + cluster
// pointers) to be written to the card. FatFile::close() flushes the file's own
// data buffer, but the directory entry that records a file's size lives in a
// separate RAM cache that FatFs otherwise only commits on the next directory
// operation or on unmount. Because we close the log and then just sit idle,
// that directory update never reaches the card, so the file shows as 0 bytes
// when the card is read on a computer. Unmounting and remounting forces the
// commit. The Core2 mounts the microSD on CS pin 4 (see ensureSd()).
void GyroLogWriter::syncVolume()
{
    if(_sd.fatType() == 0)
        return; // not mounted

    _sd.end();
    SPI.begin();
    _sd.begin(SdSpiConfig(4, SHARED_SPI | USER_SPI_BEGIN, SD_SCK_MHZ(20)));
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
    DEBUG_INFO("[GYRO-DIAG] begin(): SD ready, fatType=%d", (int)_sd.fatType());

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

    // Open the GCSV file with SdFat. SdFat paths are relative to the volume
    // root, so "/<name>.gcsv" is correct (no "/sd" VFS prefix). O_CREAT|O_TRUNC
    // creates/truncates the file.
    uint32_t tOpen = micros();
    if(!_file.open(_gcsvPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC))
    {
        DEBUG_INFO("[GYRO-DIAG] begin(): FatFile::open FAILED for '%s' (err=%d)", _gcsvPath.c_str(), (int)_file.getError());
        return false;
    }
    DEBUG_INFO("[GYRO-DIAG] begin(): open() took %lu us", (unsigned long)(micros() - tOpen));

    // Wire the (PSRAM-backed) ring buffer to the file. The sampler writes rows
    // into the ring; drainRing() calls ring.writeOut() to commit them to the file.
    // If the PSRAM allocation fails we can't decouple the sampler from the writer,
    // so abort the start (the file is left open but we return false).
    if(!_ring.begin(&_file, kRingSize))
    {
        DEBUG_ERROR("[GYRO-DIAG] begin(): PSRAM ring alloc (%lu KB) FAILED -- cannot start logger",
            (unsigned long)(kRingSize / 1024));
        _file.close();
        return false;
    }

    // Configure the MPU6886 for 1 kHz output-register polling (sets _tscale = 1
    // ms). Done before the header is written so the tscale is in the file.
    configurePolling();

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
        "tscale,%.6f\n"
        "gscale,%.11f\n"
        "ascale,%.11f\n"
        "t,gx,gy,gz,ax,ay,az\n",
        (double)_tscale,
        kGscale,
        kAscale);

    // Write the header straight to the file (before any samples). A single
    // FatFile::write of the whole header is fine -- it's a few hundred bytes.
    // (The writer task is not started yet, so this is the only writer right now.)
    _file.write((const uint8_t*)header, (size_t)n);

    // Create the ring's cross-task synchronization objects and start both tasks:
    // the sampler task (reads the IMU into the ring at 1 kHz) and the writer task
    // (commits the ring to the card). Starting the sampler AFTER the header is
    // written means no samples are lost before the header.
    if(!_ringMutex)
        _ringMutex = xSemaphoreCreateMutex();
    if(!_dataSem)
        _dataSem = xSemaphoreCreateBinary();
    _writerStop = false;
    _samplerStop = false;
    startWriterTask();
    startSamplerTask();

    _tMs = 0;
    _startMicros = micros();
    _lastSampleMicros = _startMicros;
    _lastDrainMicros = _startMicros;
    _fifoSeq = 0; // GCSV "t" starts at 0
    _i2cFailures = 0;

    _state = State::Recording;
    // Wake the (persistent) writer task so it sees the new state and starts
    // committing the ring. The sampler task polls _state itself (5 ms) so it
    // needs no explicit wake.
    if(_dataSem)
        xSemaphoreGive(_dataSem);
    return true;
}

void GyroLogWriter::poll()
{
    // The dedicated sampler task does all the IMU sampling (it runs fast enough
    // to hold a clean 1 kHz). Nothing to do here on the main loop -- this is kept
    // as a no-op hook so the call site in loop() stays simple.
    (void)_state;
}

void GyroLogWriter::drainRing()
{
    if(!_file.isOpen())
        return;

    // Two-phase drain so the ring mutex is held only for a FAST copy, not the
    // slow SD write. Phase 1 (under the mutex): copy the buffered bytes out of the
    // PSRAM ring into a local RAM buffer. Phase 2 (NO mutex): do the slow
    // FatFile::write() on the local buffer. Holding the mutex only for the copy
    // means the sampler's ring appends (which also need the mutex) are never
    // blocked by a multi-ms card write -- that blocking was what made the 1 kHz
    // sampler miss ~10% of its ticks.
    size_t used = _ring.bytesUsed();
    if(used == 0)
        return;

    // A local buffer big enough for the largest batch we'll copy at once. The
    // writer commits in kMinWriteBytes (80 KB) batches, so 96 KB is ample. This
    // is allocated once (static) and reused.
    static uint8_t* s_scratch = nullptr;
    static size_t s_scratchSize = 0;
    if(s_scratchSize < used)
    {
        if(s_scratch)
            free(s_scratch);
        s_scratchSize = used < (96 * 1024) ? (96 * 1024) : used;
        s_scratch = (uint8_t*)malloc(s_scratchSize);
        if(!s_scratch)
            return; // can't allocate; skip this drain (data stays in the ring)
    }

    // Phase 1: fast copy out of the ring, under the mutex.
    if(xSemaphoreTake(_ringMutex, portMAX_DELAY) != pdTRUE)
        return;
    size_t copied = _ring.copyOut(s_scratch, used);
    xSemaphoreGive(_ringMutex);

    // Phase 2: slow SD write of the copied bytes, WITHOUT the mutex.
#if GYRO_SKIP_SD_WRITE
    (void)copied; // diagnostic: skip the card write
#else
    if(copied > 0)
        _file.write(s_scratch, copied);
#endif
}

// The writer task's main loop. It waits (with a short timeout) for the sampler
// to append rows to the ring, then drains the ring to the file. Running this on
// its own task -- on the other core -- is what lets the sampler hold a true
// 1 kHz cadence: a card write takes tens of ms and would otherwise stall the
// sampler. The timeout (rather than a pure semaphore wait) also lets the task
// notice a stop request promptly and keeps draining any stragglers.
void GyroLogWriter::writerTaskTrampoline(void* param)
{
    ((GyroLogWriter*)param)->writerTask();
    // The task is persistent: writerTask() loops forever and never returns.
}

void GyroLogWriter::writerTask()
{
    _lastWriteMicros = micros();
    for(;;)
    {
        // Not recording: sleep on the semaphore (woken by begin()/end() giving
        // it, or by the 20 ms timeout). This is the idle state between clips.
        if(_state != State::Recording)
        {
            xSemaphoreTake(_dataSem, pdMS_TO_TICKS(20));
            continue;
        }

#if GYRO_SKIP_WRITER_WORK
        // [DIAG] do nothing (no ring drain, no SD write) -- just sleep a tick.
        // Tests whether the writer's work is the leak source.
        vTaskDelay(1);
        continue;
#endif

        // Wait for the sampler to append data, or for the poll timeout. The 20 ms
        // timeout keeps the stop/interval checks responsive without busy-spinning.
        if(xSemaphoreTake(_dataSem, pdMS_TO_TICKS(20)) != pdTRUE)
            continue;

        // Decide whether to write now: a big enough chunk is buffered, or we've
        // waited the max interval since the last write (so a slow trickle still
        // gets flushed). This batches the SPI transactions so they don't disturb
        // the I2C IMU read on the other core.
        uint32_t now = micros();
        size_t used;
        if(xSemaphoreTake(_ringMutex, portMAX_DELAY) != pdTRUE)
            continue;
        used = _ring.bytesUsed();
        xSemaphoreGive(_ringMutex);

        bool bigEnough = (used >= kMinWriteBytes);
        bool intervalElapsed = (now - _lastWriteMicros >= kMaxWriteIntervalUs);
        if(!bigEnough && !intervalElapsed)
            continue; // not yet; leave the data in the ring and keep waiting

        drainRing();
        _lastWriteMicros = micros();
    }
}

void GyroLogWriter::startWriterTask()
{
    if(_writerTask != nullptr)
        return; // already running
    _writerStop = false;
    // 8 KB stack: the task only does a FatFile::write of a few KB, so this is
    // ample. Pinned to CORE 1 at priority 1 (same as the main loop and the
    // sampler).
    //
    // Both the sampler and the writer live on core 1 at priority 1 (the Arduino
    // main loop's priority), so all three time-slice fairly within each tick --
    // none starves the others. (A higher-priority sampler/writer on core 1 hung
    // the main loop; a sampler on core 0 starved the BLE idle task and tripped
    // the task watchdog.) The writer only wakes every ~100 ms to flush a 16 KB
    // batch, so its brief SD writes don't meaningfully disturb the 1 kHz sampler;
    // the 128 KB ring absorbs the rest.
    xTaskCreatePinnedToCore(&GyroLogWriter::writerTaskTrampoline, "gyroWriter", 8192,
        this, 1, &_writerTask, 1);
}

void GyroLogWriter::stopWriterTask()
{
    // The writer task is persistent -- we don't delete it. We just wake it (give
    // _dataSem) so it notices _state != Recording (set by end()) and returns to
    // its idle sleep. The final ring drain is done by end() directly (drainRing),
    // so no data is lost.
    if(_dataSem)
        xSemaphoreGive(_dataSem);
}

// The sampler task's main loop. It is PERSISTENT (created once, never deleted)
// so we don't churn the tight internal heap with a 4 KB alloc/free per recording
// (that fragmentation made the 2nd recording's xTaskCreate fail). When not
// recording it sleeps; when a recording starts (begin() sets _state=Recording) it
// wakes and polls the MPU6886 output registers once per 1 ms tick -- a clean,
// dense 1 kHz sample stream.
void GyroLogWriter::samplerTaskTrampoline(void* param)
{
    ((GyroLogWriter*)param)->samplerTask();
    // The task is persistent: it never returns. (samplerTask() loops forever.)
}

void GyroLogWriter::samplerTask()
{
    // 1 kHz sampling pinned to the REAL-TIME 1 ms grid (esp_timer_get_time,
    // microsecond-accurate). Each iteration we:
    //   1. spin (with tiny yields so the writer task gets scheduled) until the
    //      next 1 ms boundary,
    //   2. read the latest sample exactly on the boundary and append one dense row,
    //   3. if the read finished with time to spare before the NEXT boundary, yield
    //      a bit so the writer can commit the ring; otherwise go straight to the
    //      next boundary.
    // Pinning to the grid (rather than a vTaskDelay(1) tick) means a slow I2C read
    // or a brief scheduler hiccup does NOT accumulate drift: the next sample is
    // always at the next 1 ms boundary, so the long-run rate is exactly 1 kHz and
    // the "t" index stays dense. The I2C read takes ~0.4-0.6 ms (well under 1 ms),
    // so there's always slack to yield to the writer without missing the grid.
    uint64_t nextBoundaryUs = 0;
    for(;;)
    {
        // Not recording: poll _state every ~5 ms (cheap) until a recording
        // starts. We don't block on a semaphore here because _dataSem is shared
        // with the writer task; a short poll is simpler and the 5 ms wake latency
        // is fine (a recording start is a deliberate user action).
        if(_state != State::Recording)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            nextBoundaryUs = 0; // resync the grid on (re)start
            continue;
        }

#if GYRO_SKIP_SAMPLER_WORK
        // [DIAG] do nothing per tick (no ring write, no semaphore) -- just sleep a
        // tick. Tests whether the sampler's per-tick work is the leak source.
        vTaskDelay(1);
        continue;
#endif

        // Compute the next 1 ms boundary (in esp_timer us) if we haven't yet.
        if(nextBoundaryUs == 0)
        {
            uint64_t now = esp_timer_get_time();
            nextBoundaryUs = ((now / 1000) + 1) * 1000;
        }

        // Wait until the next 1 ms boundary. Two phases:
        //   1. While there's more than one tick (~1 ms) to the boundary, sleep a
        //      tick (vTaskDelay). This is the crucial part: a pure spin here would
        //      starve the CPU-0 idle task (IDLE0, priority 0) -- the sampler runs
        //      at priority 5 and portYIELD() only yields to equal/higher-priority
        //      tasks, never to IDLE0 -- and the task watchdog (which needs the idle
        //      task to run) aborts the CPU after ~35 s. Sleeping a tick lets IDLE0
        //      run and keeps the watchdog happy.
        //   2. In the final sub-tick window (<= ~1 ms to the boundary), tight-spin
        //      on the microsecond timer for microsecond accuracy (a vTaskDelay here
        //      would overshoot the boundary by up to a full tick and make the read
        //      start late).
        while(true)
        {
            int64_t now = esp_timer_get_time();
            int64_t toGo = nextBoundaryUs - now;
            if(toGo <= 0)
                break;
            if(toGo > 1000) // > 1 ms to go: sleep a tick so IDLE0 runs
                vTaskDelay(1);
            else
                portYIELD(); // final sub-tick: tight spin for accuracy
        }

        // Read the latest sample exactly on the boundary and append one dense row.
        uint32_t tRead0 = micros();
        pollOutputRegisters();
        uint32_t readUs = micros() - tRead0;

        // [DIAG] Once per second, report the loop rate and the I2C read time
        // distribution, to see how close to 1 kHz we are and where time goes.
        // Only compiled in when GYROLOG_DEBUG is on (off by default).
#if GYROLOG_DEBUG
        {
            static uint32_t dLast = 0;
            static uint32_t dLoops = 0, dReadUs = 0, dMaxRead = 0;
            dLoops++;
            dReadUs += readUs;
            if(readUs > dMaxRead) dMaxRead = readUs;
            uint32_t now = millis();
            if(dLast == 0) dLast = now;
            if(now - dLast >= 1000)
            {
                DEBUG_INFO("[GYRO-DIAG] sampler: %lu loops/s (want ~1000), avg I2C read %lu us, max %lu us",
                    (unsigned long)dLoops, (unsigned long)(dLoops ? dReadUs / dLoops : 0), (unsigned long)dMaxRead);
                dLast = now; dLoops = 0; dReadUs = 0; dMaxRead = 0;
            }
        }
#else
        (void)readUs;
#endif

        // Advance to the next 1 ms boundary. The next iteration's wait (above)
        // sleeps a tick while there's slack (so IDLE0 runs and the watchdog stays
        // happy) and tight-spins only in the final sub-tick window for accuracy.
        // The writer (priority 6) still runs: it's woken by _dataSem and preempts
        // the sampler briefly to flush the ring; the 128 KB ring absorbs that.
        nextBoundaryUs += 1000;
    }
}

void GyroLogWriter::startSamplerTask()
{
    if(_samplerTask != nullptr)
        return; // already created (persistent task)
    // 4 KB stack: the task only does a small I2C read and a ring append, so
    // this is ample. Pinned to CORE 1 at priority 1 (the same priority as the
    // Arduino main loop).
    //
    // Core choice: core 1, NOT core 0. The BLE controller task (BTC_TASK) runs
    // on core 0; a sampler there starves the core-0 idle task (IDLE0) and the
    // task watchdog aborts the CPU after ~35 s.
    //
    // Priority choice: 1 (== the main loop's priority), NOT higher. A sampler at
    // a higher priority on the same core as the main loop preempts it and the
    // main loop never runs (it hung after the record started). At equal priority
    // FreeRTOS time-slices the two within each tick, so the main loop and the
    // sampler both run. The sampler's wait sleeps a tick while there's slack
    // (letting the main loop run) and tight-spins only in the final sub-tick
    // window for grid accuracy.
    BaseType_t rc = xTaskCreatePinnedToCore(&GyroLogWriter::samplerTaskTrampoline, "gyroSampler", 4096,
        this, 2, &_samplerTask, 1);
    if(rc != pdPASS || _samplerTask == nullptr)
        DEBUG_ERROR("[GYRO-DIAG] startSamplerTask(): FAILED rc=%d (freeHeap=%lu, freePsram=%lu) -- sampler will NOT run",
            (int)rc, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
    else
        DEBUG_INFO("[GYRO-DIAG] startSamplerTask(): OK (core 1, prio 1, persistent)");
}

void GyroLogWriter::stopSamplerTask()
{
    // The sampler task is persistent -- we don't delete it. We just make it sleep
    // by clearing the recording state (done in end()) and wake it so it notices.
    // We do one final output-register read here (on the calling task) to flush the
    // very end of the clip into the ring before the writer commits it.
    pollOutputRegisters();
    if(_dataSem)
        xSemaphoreGive(_dataSem); // wake the sampler so it sees _state != Recording
}

void GyroLogWriter::closeFile()
{
    if(_file.isOpen())
    {
        // Flush the file's data and its directory entry (size) to the card.
        _file.sync();
        _file.close();
    }
}

// Count the data rows in a closed GCSV file. A data row is a line whose first
// character is a digit (the "t" sample index). Header lines ("GYROFLOW IMU
// LOG", "version,...", "t,gx,gy,..." etc.) all start with a letter, so this
// cleanly separates the ~N sample rows from the header. Reads the whole file
// in chunks (it's a few hundred KB at most) and counts newlines that begin a
// digit-leading line.
long GyroLogWriter::countSamplesInFile(const std::string& path)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return -1;

    long count = 0;
    uint8_t buf[512];
    size_t n;
    bool lineStart = true; // are we at the start of a line?
    while((n = f.read(buf, sizeof(buf))) > 0)
    {
        for(size_t i = 0; i < n; i++)
        {
            if(lineStart)
            {
                // The first byte of a line: a digit means a sample row.
                if(buf[i] >= '0' && buf[i] <= '9')
                    count++;
                lineStart = false;
            }
            if(buf[i] == '\n')
                lineStart = true;
        }
    }
    f.close();
    return count;
}

// Dump the first n bytes of a file to the debug log in small chunks, so we can
// confirm the GCSV header and first sample rows are well-formed.
void GyroLogWriter::dumpFileHead(const std::string& path, size_t n)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
    {
        DEBUG_ERROR("[GYRO-E2E] dumpFileHead: could not open '%s'", path.c_str());
        return;
    }
    char line[192];
    size_t total = 0;
    int c;
    size_t lineLen = 0;
    while(total < n && (c = f.read()) != -1)
    {
        if(c == '\n')
        {
            line[lineLen] = 0;
            DEBUG_INFO("[GYRO-E2E]   |%s", line);
            lineLen = 0;
        }
        else if(lineLen < sizeof(line) - 1)
        {
            line[lineLen++] = (char)c;
        }
        total++;
    }
    if(lineLen > 0)
    {
        line[lineLen] = 0;
        DEBUG_INFO("[GYRO-E2E]   |%s", line);
    }
    f.close();
    DEBUG_INFO("[GYRO-E2E] dumpFileHead: printed %lu bytes of '%s'", (unsigned long)total, path.c_str());
}

// Scan the "t" index of a closed GCSV file and report its health for Gyroflow.
// A clean file has t = 0,1,2,...,N-1 (maxGap=1, no backwards steps). Dropped
// samples show up as gaps (maxGap>1); a FIFO-overflow reset shows up as t going
// backwards (which would corrupt Gyroflow's timeline). This is the definitive
// check that the timeline is usable, not just that samples exist.
void GyroLogWriter::analyzeTIndex(const std::string& path)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
    {
        DEBUG_ERROR("[GYRO-E2E] analyzeTIndex: could not open '%s'", path.c_str());
        return;
    }

    // Read the file line by line. For each sample row (first char a digit),
    // parse the leading integer (the "t" value) and track first/last/maxGap and
    // whether t ever decreases.
    long firstT = -1;
    long lastT = -1;
    long prevT = -1;
    long maxGap = 0;
    long backwards = 0;
    long rows = 0;

    uint8_t buf[512];
    size_t n;
    // Accumulate the current line; a line is delimited by '\n'.
    char line[64];
    size_t lineLen = 0;
    auto processLine = [&](const char* s)
    {
        if(s[0] < '0' || s[0] > '9')
            return; // not a sample row (header line)
        // Parse the leading integer.
        long val = 0;
        size_t i = 0;
        while(s[i] >= '0' && s[i] <= '9')
        {
            val = val * 10 + (s[i] - '0');
            i++;
        }
        rows++;
        if(firstT < 0)
            firstT = val;
        lastT = val;
        if(prevT >= 0)
        {
            long gap = val - prevT;
            if(gap > maxGap)
                maxGap = gap;
            if(gap < 0)
                backwards++;
        }
        prevT = val;
    };

    while((n = f.read(buf, sizeof(buf))) > 0)
    {
        for(size_t i = 0; i < n; i++)
        {
            if(buf[i] == '\n')
            {
                line[lineLen] = 0;
                processLine(line);
                lineLen = 0;
            }
            else if(lineLen < sizeof(line) - 1)
            {
                line[lineLen++] = (char)buf[i];
            }
        }
    }
    if(lineLen > 0)
    {
        line[lineLen] = 0;
        processLine(line);
    }
    f.close();

    DEBUG_INFO("[GYRO-E2E] t-index: rows=%ld first=%ld last=%ld maxGap=%ld backwards=%ld -> %s",
        (long)rows, (long)firstT, (long)lastT, (long)maxGap, (long)backwards,
        (backwards == 0 && maxGap <= 2) ? "OK (Gyroflow-usable)" : "DEGRADED (gaps/resets)");
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
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return false;

    size_t size = f.fileSize();
    std::string content;
    content.reserve(size + 64);
    char buf[256];
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
    FatFile w;
    if(!w.open(path.c_str(), O_WRONLY | O_TRUNC))
        return false;
    w.write((const uint8_t*)out.data(), out.size());
    w.close();
    return true;
}

// Rewrite the "timestamp" header line of a closed .gcsv file at `path` to the
// given UNIX epoch. Same read-modify-write approach as rewriteVideoFileName().
bool GyroLogWriter::rewriteTimestamp(const std::string& path, long epoch)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return false;

    size_t size = f.fileSize();
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

    FatFile w;
    if(!w.open(path.c_str(), O_WRONLY | O_TRUNC))
        return false;
    w.write((const uint8_t*)out.data(), out.size());
    w.close();
    return true;
}

// Set the FAT modification time of the file at `path` to `epoch` (UNIX
// seconds) using SdFat's FatFile::timestamp(). The caller sets the system clock
// to the clip's date first, so the timestamp we pass matches it. syncVolume()
// then commits the directory entry to the card.
bool GyroLogWriter::setFileMtime(const std::string& path, long epoch)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return false;

    // Convert the UNIX epoch to calendar components.
    time_t t = (time_t)epoch;
    struct tm tmv;
    localtime_r(&t, &tmv);

    bool ok = f.timestamp(T_WRITE, (uint16_t)(tmv.tm_year + 1900), (uint8_t)(tmv.tm_mon + 1),
        (uint8_t)tmv.tm_mday, (uint8_t)tmv.tm_hour, (uint8_t)tmv.tm_min, (uint8_t)tmv.tm_sec);
    f.close();
    return ok;
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
    // card) by end(). Move it to the real clip name with SdFat's rename().
    if(std::strcmp(oldPath, newPath) != 0)
    {
        // Diagnose the rename: is the source file even there after end()/remount?
        DEBUG_INFO("[GYRO-RENAME] pre-rename: exists('%s')=%d exists('%s')=%d fatType=%d",
            oldPath, (int)_sd.exists(oldPath), newPath, (int)_sd.exists(newPath), (int)_sd.fatType());
        bool renamed = _sd.rename(oldPath, newPath);
        DEBUG_INFO("[GYRO-RENAME] rename '%s' -> '%s' ok=%d (after: exists old=%d new=%d)",
            oldPath, newPath, (int)renamed, (int)_sd.exists(oldPath), (int)_sd.exists(newPath));
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

    // [DIAG] Timestamp each step of end() to find where a long block happens.
    uint32_t tEnter = micros();
    DEBUG_INFO("[GYRO-DIAG] end(): ENTER (t=%lu us)", (unsigned long)tEnter);

    // The sampler and writer tasks are PERSISTENT (created once, never deleted).
    // To stop recording we (1) flip _state to Idle so both tasks return to their
    // idle sleep, (2) do a final IMU read on this thread to capture the very last
    // sample, and (3) drain the ring to the file. No task is deleted, so we don't
    // churn the tight internal heap (the per-recording task create/delete was
    // fragmenting the heap and making the 2nd recording's xTaskCreate fail).
    _state = State::Idle; // both tasks see this and stop working / go to sleep
    stopSamplerTask();     // final IMU read + wake the sampler so it sleeps
    DEBUG_INFO("[GYRO-DIAG] end(): stopSamplerTask took %lu us", (unsigned long)(micros() - tEnter));
    tEnter = micros();
    stopWriterTask();       // wake the writer so it returns to idle sleep
    DEBUG_INFO("[GYRO-DIAG] end(): stopWriterTask took %lu us", (unsigned long)(micros() - tEnter));
    tEnter = micros();

    // Final drain of any ring stragglers (both tasks are idle now), then close
    // the file and commit the directory entry.
    drainRing();
    DEBUG_INFO("[GYRO-DIAG] end(): drainRing took %lu us", (unsigned long)(micros() - tEnter));
    tEnter = micros();

    // Restore the MPU6886 to a clean state now that we're done sampling: clear
    // the FIFO-enable (a no-op in polling mode, but harmless) and put the clock
    // source back to M5Unified's default (the 8 MHz RC, PWR_MGMT_1=0x01) so the
    // calibration screen's readImu() and any later M5Unified use see the sensor the
    // way they expect. The output registers (0x3B+) still hold the latest sample
    // regardless, so this only affects the clock/FIFO config.
    M5.In_I2C.writeRegister8(kImuAddr, 0x6A, 0x00, 400000); // FIFO_EN = 0 (disable FIFO)
    M5.In_I2C.writeRegister8(kImuAddr, 0x6B, 0x01, 400000); // clock src = 8 MHz RC (M5Unified default)
    _fifoConfigured = false;
    DEBUG_INFO("[GYRO-DIAG] end(): sensor-restore (I2C) took %lu us", (unsigned long)(micros() - tEnter));
    tEnter = micros();

    if(_file.isOpen())
    {
        _finalFileSizeBytes = _file.fileSize();
        uint32_t tClose = micros();
        closeFile(); // truncate() + sync() + close()
        DEBUG_INFO("[GYRO-DIAG] end(): closeFile() (sync+close) took %lu us", (unsigned long)(micros() - tClose));
        uint32_t tSync = micros();
        syncVolume();
        DEBUG_INFO("[GYRO-DIAG] end(): syncVolume() (unmount+remount) took %lu us", (unsigned long)(micros() - tSync));
    }

    // Capture the summary. The clip duration is the number of samples captured
    // (_fifoSeq) times the measured seconds-per-sample (_tscale), in ms. This is
    // the true data duration (independent of any wall-clock stalls).
    _summary.valid = true;
    _summary.fileName = _startedName + ".gcsv";
    _summary.videoFileName = _videoFileName;
    _summary.durationMs = (uint32_t)((double)_fifoSeq * _tscale * 1000.0);
    _summary.fileSizeBytes = _finalFileSizeBytes;
    // Total space from the volume's cluster count (cheap: read from the FAT boot
    // sector, no FAT walk). We deliberately do NOT call freeClusterCount() here:
    // on a FAT32 card it walks the entire FAT to count free clusters, which took
    // ~99 s on this card and hung the whole stop path. Free space is a
    // nice-to-have for the on-screen summary, not essential, so we leave it 0
    // rather than block the UI for a minute.
    uint32_t bpc = _sd.bytesPerCluster();
    _summary.totalBytes = (uint64_t)_sd.clusterCount() * bpc;
    _summary.freeBytes = 0;

    _state = State::Idle;
    DEBUG_INFO("[GYRO-DIAG] end(): LEAVE (summary step took %lu us)", (unsigned long)(micros() - tEnter));
    return true;
}
