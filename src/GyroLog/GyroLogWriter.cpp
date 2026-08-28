#include "GyroLogWriter.h"
#include <SPI.h>
#include <cstring> // strcmp
#include <M5Unified.h> // M5.In_I2C (MPU6886 output-register reads)
#include <nvs.h>
#include <esp_timer.h> // esp_timer_get_time() for the microsecond-accurate 1 kHz sampling grid
#include <math.h>
#include <time.h>

// 1 = keep the per-second sampler diagnostic (loop rate + I2C read time). 0 =
// quiet (the default) so the 1 kHz sampling path does no per-second logging.
#ifndef GYROLOG_DEBUG
#define GYROLOG_DEBUG 1
#endif

// The 24 GCSV orientation tokens, indexed by orientation index (0..23).
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

// gscale: raw gyro (deg/s) -> rad/s. MPU6886 is configured +/-2000 deg/s.
static const float kGscale = (2000.0f * 3.141592653589793f / 180.0f) / 32768.0f;
// ascale: raw accel (g) -> g. MPU6886 is configured +/-8 g.
static const float kAscale = 8.0f / 32768.0f;

// The year assumed for a clip's date until the real date is learned from the
// clip's file name (the name only carries MMDDHHMM, no year).
static const int kDefaultYear = 2026;

// Turn a raw clip/slate name into a safe file-name base. Placeholder slate names
// the camera sends (e.g. "Next Clip") are dropped (returns empty), and any
// characters that are unsafe in a file name are replaced with '_'.
static std::string sanitiseClipName(const std::string& name)
{
    size_t b = name.find_first_not_of(" \t");
    size_t e = name.find_last_not_of(" \t");
    if(b == std::string::npos)
        return ""; // blank
    std::string trimmed = name.substr(b, e - b + 1);

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

// Mount the SD card via SdFat. The Core2's microSD slot is on the SPI bus with
// its chip-select on GPIO4. We share the SPI bus with the M5GFX display
// (SHARED_SPI), so SdFat toggles only the SD's CS pin. The display already
// initialised the VSPI bus, so we tell SdFat NOT to re-begin it
// (USER_SPI_BEGIN) -- re-initialising an active SPI bus corrupts the MISO path.
// 20 MHz is a safe, verified clock for the card.
bool GyroLogWriter::ensureSd()
{
    if(_sd.fatType() != 0)
    {
        _sdReady = true;
        _sdStatusMessage = "ready";
        return true;
    }

    SPI.begin();
    if(!_sd.begin(SdSpiConfig(4, SHARED_SPI | USER_SPI_BEGIN, SD_SCK_MHZ(20))))
    {
        _sdReady = false;
        _sdStatusMessage = "mount failed (no card / not FAT?)";
        DEBUG_INFO("[GYRO] ensureSd: SdFat begin FAILED cardType=%d sdErrorCode=0x%02X",
            (int)_sd.card()->type(), (unsigned)_sd.sdErrorCode());
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
// data, but the directory entry that records a file's size lives in a separate
// RAM cache that FatFs otherwise only commits on unmount.
void GyroLogWriter::syncVolume()
{
    if(_sd.fatType() == 0)
        return; // not mounted

    _sd.end();
    SPI.begin();
    _sd.begin(SdSpiConfig(4, SHARED_SPI | USER_SPI_BEGIN, SD_SCK_MHZ(20)));
}

void GyroLogWriter::closeFile()
{
    if(_file.isOpen())
    {
        _file.sync();
        _file.close();
    }
}

// Configure the sensor for OUTPUT-REGISTER polling at ~1 kHz. We wake the sensor
// (PWR_MGMT_1: CLKSEL=001 auto-select PLL gyro clock, SLEEP clear) and set the
// DLPF/SMPLRT_DIV so the output registers refresh at a known rate. We do NOT
// enable the FIFO -- we read the output registers (0x3B+) directly at 1 kHz from
// the sampler task. This keeps the I2C load trivial (~14 KB/s) and the "t" index
// dense by construction, avoiding the FIFO's ~2.3 kHz rate that the I2C bus
// can't drain losslessly (and which we measured to be ~59% duplicated).
void GyroLogWriter::configurePolling()
{
    // Wake the sensor and select the clock source. Per the MPU-6886 datasheet
    // (DS-000193 v1.1) section 9.6, CLKSEL[2:0] MUST be 001 (auto-select: PLL
    // gyro clock if ready, else internal oscillator) for full performance.
    // 0x01 sets CLKSEL=001 and leaves SLEEP/CYCLE/GYRO_STANDBY/TEMP_DIS clear.
    M5.In_I2C.writeRegister8(kImuAddr, 0x6B, 0x01, 400000);
    vTaskDelay(pdMS_TO_TICKS(10)); // let the PLL lock

    // DLPF (CONFIG 0x1A) = 0x01 (44 Hz) and SMPLRT_DIV (0x19) = 0. The exact
    // sensor ODR doesn't matter for a 1 kHz poll -- we just need the output
    // registers to hold a fresh sample, which they do at any ODR >= 1 kHz.
    M5.In_I2C.writeRegister8(kImuAddr, 0x19, 0x00, 400000); // SMPLRT_DIV = 0
    M5.In_I2C.writeRegister8(kImuAddr, 0x1A, 0x01, 400000); // CONFIG: DLPF = 0x01
    // Make sure the FIFO is disabled (we're not using it in this mode).
    M5.In_I2C.writeRegister8(kImuAddr, 0x6A, 0x00, 400000); // USER_CTRL: FIFO_EN = 0

    // For the 1 kHz poll mode the output rate is exactly our poll rate (1 kHz),
    // so tscale is simply 1 ms per sample. We set it here (not measured) because
    // the poll cadence -- not the sensor's internal ODR -- defines the sample
    // spacing in the file.
    _tscale = 0.001f; // 1 ms per sample (1 kHz)
    DEBUG_INFO("[GYRO] configurePolling(): 1 kHz output-register poll mode, tscale=0.001000 s");
    _fifoConfigured = true;
}

// Read the latest gyro+accel sample from the output registers and append one
// dense GCSV row to the ring. The output registers (0x3B..0x48) always hold the
// most recent sample: accel X/Y/Z (6 B), temp (2 B), gyro X/Y/Z (6 B) = 14
// bytes. We read them in a single I2C transaction and write one row with the
// running _fifoSeq as the "t" index. Because the sampler calls this exactly once
// per 1 ms tick, the "t" index is dense (0,1,2,...) and the timeline is accurate.
uint32_t GyroLogWriter::pollOutputRegisters()
{
    uint8_t buf[14];

    // Read 14 bytes starting at 0x3B (the output-register block). Retry a couple
    // times on an I2C glitch (a single atomic read; a failure means no data was
    // transferred, so a retry re-reads the same latest sample).
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

    if(xSemaphoreTake(_ringMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        _ring.write((const uint8_t*)row, (size_t)n);
        xSemaphoreGive(_ringMutex);
        if(_dataSem)
            xSemaphoreGive(_dataSem); // wake the writer
    }
    _fifoSeq++;
    return 1;
}

// Drain as much of the ring buffer as possible to the file. Two-phase so the
// ring mutex is held only for a FAST copy, not the slow SD write: Phase 1
// (under the mutex) copies the buffered bytes out of the PSRAM ring into a local
// RAM buffer; Phase 2 (NO mutex) does the slow FatFile::write() on the local
// buffer. Holding the mutex only for the copy means the sampler's ring appends
// are never blocked by a multi-ms card write.
void GyroLogWriter::drainRing()
{
    if(!_file.isOpen())
        return;

    size_t used;
    if(xSemaphoreTake(_ringMutex, portMAX_DELAY) != pdTRUE)
        return;
    used = _ring.bytesUsed();
    xSemaphoreGive(_ringMutex);
    if(used == 0)
        return;

    // A local buffer big enough for the largest batch we'll copy at once. The
    // writer commits in kMinWriteBytes (16 KB) batches, so 32 KB is ample. This
    // is allocated once (static) and reused.
    static uint8_t* s_scratch = nullptr;
    static size_t s_scratchSize = 0;
    if(s_scratchSize < used)
    {
        if(s_scratch)
            free(s_scratch);
        s_scratchSize = used < (32 * 1024) ? (32 * 1024) : used;
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
    if(copied > 0)
        _file.write(s_scratch, copied);
}

// The writer task's main loop. It waits (with a short timeout) for the sampler
// to append rows to the ring, then drains the ring to the file. Running this on
// its own task -- on the other core -- is what lets the sampler hold a true 1 kHz
// cadence: a card write takes tens of ms and would otherwise stall the sampler.
void GyroLogWriter::writerTaskTrampoline(void* param)
{
    ((GyroLogWriter*)param)->writerTask();
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
    // 8 KB stack: the task only does a FatFile::write of a few KB. Pinned to
    // CORE 1 at priority 1 (same as the main loop and the sampler) so all three
    // time-slice fairly within each tick.
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
// so we don't churn the tight internal heap with a 4 KB alloc/free per recording.
// When not recording it sleeps; when a recording starts (begin() sets
// _state=Recording) it wakes and polls the MPU6886 output registers once per 1
// ms tick -- a clean, dense 1 kHz sample stream.
void GyroLogWriter::samplerTaskTrampoline(void* param)
{
    ((GyroLogWriter*)param)->samplerTask();
}

void GyroLogWriter::samplerTask()
{
    // 1 kHz sampling pinned to the REAL-TIME 1 ms grid (esp_timer_get_time,
    // microsecond-accurate). Each iteration we:
    //   1. spin (with tiny yields so the writer task gets scheduled) until the
    //      next 1 ms boundary,
    //   2. read the latest sample exactly on the boundary and append one dense row,
    //   3. advance to the next boundary.
    // Pinning to the grid (rather than a vTaskDelay(1) tick) means a slow I2C
    // read or a brief scheduler hiccup does NOT accumulate drift: the next sample
    // is always at the next 1 ms boundary, so the long-run rate is exactly 1 kHz
    // and the "t" index stays dense.
    uint64_t nextBoundaryUs = 0;
    for(;;)
    {
        // Not recording: poll _state every ~5 ms (cheap) until a recording starts.
        if(_state != State::Recording)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            nextBoundaryUs = 0; // resync the grid on (re)start
            continue;
        }

        // Compute the next 1 ms boundary (in esp_timer us) if we haven't yet.
        if(nextBoundaryUs == 0)
        {
            uint64_t now = esp_timer_get_time();
            nextBoundaryUs = ((now / 1000) + 1) * 1000;
        }

        // Wait until the next 1 ms boundary. Two phases:
        //   1. While there's more than one tick (~1 ms) to the boundary, sleep a
        //      tick (vTaskDelay). A pure spin here would starve the CPU idle task
        //      and trip the task watchdog; sleeping a tick keeps it happy.
        //   2. In the final sub-tick window (<= ~1 ms to the boundary), tight-spin
        //      on the microsecond timer for microsecond accuracy (a vTaskDelay
        //      here would overshoot the boundary by up to a full tick).
        while(true)
        {
            int64_t now = esp_timer_get_time();
            int64_t toGo = nextBoundaryUs - now;
            if(toGo <= 0)
                break;
            if(toGo > 1000) // > 1 ms to go: sleep a tick so the idle task runs
                vTaskDelay(1);
            else
                portYIELD(); // final sub-tick: tight spin for accuracy
        }

        // Read the latest sample exactly on the boundary and append one dense row.
        uint32_t tRead0 = micros();
        pollOutputRegisters();
        uint32_t readUs = micros() - tRead0;

#if GYROLOG_DEBUG
        // Once per second, report the loop rate and the I2C read time distribution,
        // to see how close to 1 kHz we are and where time goes.
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

        // Advance to the next 1 ms boundary.
        nextBoundaryUs += 1000;
    }
}

void GyroLogWriter::startSamplerTask()
{
    if(_samplerTask != nullptr)
        return; // already created (persistent task)
    // 4 KB stack: the task only does a small I2C read and a ring append. Pinned
    // to CORE 1 at priority 1 (the same priority as the Arduino main loop).
    //
    // Core choice: core 1, NOT core 0. The BLE controller task (BTC_TASK) runs
    // on core 0; a sampler there starves the core-0 idle task (IDLE0) and the
    // task watchdog aborts the CPU after ~35 s.
    //
    // Priority choice: 1 (== the main loop's priority), NOT higher. A sampler at
    // a higher priority on the same core as the main loop preempts it and the
    // main loop never runs. At equal priority FreeRTOS time-slices the two within
    // each tick, so the main loop and the sampler both run.
    BaseType_t rc = xTaskCreatePinnedToCore(&GyroLogWriter::samplerTaskTrampoline, "gyroSampler", 4096,
        this, 1, &_samplerTask, 1);
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
    // We do one final output-register read here (on the calling task) to flush
    // the very end of the clip into the ring before the writer commits it.
    pollOutputRegisters();
    if(_dataSem)
        xSemaphoreGive(_dataSem); // wake the sampler so it sees _state != Recording
}

GyroLogWriter::~GyroLogWriter()
{
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

bool GyroLogWriter::begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo)
{
    if(_state == State::Recording)
        return false; // already recording

    loadOrientation();

    // Make sure the SD card is mounted before we try to open the file.
    if(!ensureSd())
    {
        DEBUG_INFO("[GYRO] begin(): SD not ready (msg='%s')", _sdStatusMessage.c_str());
        return false;
    }

    // Sanitise the clip name (drops placeholder slate names like "Next Clip" and
    // replaces any file-name-unsafe characters). The caller falls back to a
    // generated "clip_NNNN" name when this comes back empty.
    _startedName = sanitiseClipName(clipName);
    if(_startedName.empty())
        _startedName = clipName;
    _extension = extension;
    _videoFileName = _startedName + "." + extension;
    _lensInfo = lensInfo;

    char path[128];
    snprintf(path, sizeof(path), "/%s.gcsv", _startedName.c_str());
    _gcsvPath = path;

    // Open the GCSV file with SdFat. O_CREAT|O_TRUNC creates/truncates it.
    if(!_file.open(_gcsvPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC))
    {
        DEBUG_INFO("[GYRO] begin(): open FAILED for '%s' (err=%d)", _gcsvPath.c_str(), (int)_file.getError());
        return false;
    }

    // Wire the (PSRAM-backed) ring buffer to the file. The sampler writes rows
    // into the ring; drainRing() commits them to the file. If the PSRAM
    // allocation fails we can't decouple the sampler from the writer, so abort.
    if(!_ring.begin(&_file, kRingSize))
    {
        DEBUG_ERROR("[GYRO] begin(): PSRAM ring alloc (%lu KB) FAILED -- cannot start logger",
            (unsigned long)(kRingSize / 1024));
        _file.close();
        return false;
    }

    // Configure the MPU6886 for 1 kHz output-register polling (sets _tscale = 1
    // ms). Done before the header is written so the tscale is in the file.
    configurePolling();

    // Write the GCSV header straight to the file (before any samples). A single
    // FatFile::write of the whole header is fine -- it's a few hundred bytes.
    // (The writer task is not started yet, so this is the only writer right now.)
    char header[700];
    int n = snprintf(header, sizeof(header),
        "GYROFLOW IMU LOG\n"
        "version,1.3\n"
        "id,m5stack-core2-mpu6886\n"
        "orientation,%s\n"
        "note,M5Stack Core2 gyro log ~1kHz; start TC %s\n"
        "fwversion,1.0.0\n"
        "timestamp,0\n"
        "vendor,m5stack\n"
        "videofilename,%s\n"
        "tscale,%.6f\n"
        "gscale,%.11f\n"
        "ascale,%.11f\n"
        "t,gx,gy,gz,ax,ay,az\n",
        GyroLogWriter::orientationToken(_orientationIndex),
        timecode.c_str(),
        _videoFileName.c_str(),
        (double)_tscale,
        kGscale,
        kAscale);
    if(!_lensInfo.empty())
    {
        std::string lens = _lensInfo;
        for(char& c : lens)
            if(c == ',' || c == '\n' || c == '\r')
                c = ' ';
        n += snprintf(header + n, sizeof(header) - (size_t)n, "lens_info,%s\n", lens.c_str());
    }
    _file.write((const uint8_t*)header, (size_t)n);

    // Create the ring's cross-task synchronization objects and start both tasks:
    // the sampler task (reads the IMU into the ring at 1 kHz) and the writer
    // task (commits the ring to the card). Starting the sampler AFTER the header
    // is written means no samples are lost before the header.
    if(!_ringMutex)
        _ringMutex = xSemaphoreCreateMutex();
    if(!_dataSem)
        _dataSem = xSemaphoreCreateBinary();
    _writerStop = false;
    _samplerStop = false;
    startWriterTask();
    startSamplerTask();

    _fifoSeq = 0; // GCSV "t" starts at 0
    _i2cFailures = 0;

    _state = State::Recording;
    // Wake the (persistent) writer task so it sees the new state and starts
    // committing the ring. The sampler task polls _state itself (5 ms) so it
    // needs no explicit wake.
    if(_dataSem)
        xSemaphoreGive(_dataSem);

    DEBUG_INFO("[GYRO] begin(): opened '%s', wrote %d-byte header (clip='%s')",
        _gcsvPath.c_str(), n, clipName.c_str());
    return true;
}

void GyroLogWriter::poll()
{
    // The dedicated sampler task does all the IMU sampling (it runs fast enough
    // to hold a clean 1 kHz). Nothing to do here on the main loop -- this is kept
    // as a no-op hook so the call site in loop() stays simple.
    (void)_state;
}

bool GyroLogWriter::end()
{
    if(_state != State::Recording)
        return false;

    // The sampler and writer tasks are PERSISTENT (created once, never deleted).
    // To stop recording we (1) flip _state to Idle so both tasks return to their
    // idle sleep, (2) do a final IMU read on this thread to capture the very last
    // sample, and (3) drain the ring to the file. No task is deleted, so we don't
    // churn the tight internal heap.
    _state = State::Idle; // both tasks see this and stop working / go to sleep
    stopSamplerTask();     // final IMU read + wake the sampler so it sleeps
    stopWriterTask();       // wake the writer so it returns to idle sleep

    // Final drain of any ring stragglers (both tasks are idle now), then close
    // the file and commit the directory entry.
    drainRing();

    // Restore the MPU6886 to a clean state now that we're done sampling: clear
    // the FIFO-enable and put the clock source back to M5Unified's default (the
    // 8 MHz RC, PWR_MGMT_1=0x01) so the calibration screen's readImu() and any
    // later M5Unified use see the sensor the way they expect.
    M5.In_I2C.writeRegister8(kImuAddr, 0x6A, 0x00, 400000); // USER_CTRL: FIFO_EN = 0
    M5.In_I2C.writeRegister8(kImuAddr, 0x6B, 0x01, 400000); // clock src = 8 MHz RC (M5Unified default)
    _fifoConfigured = false;

    if(_file.isOpen())
    {
        _finalFileSizeBytes = _file.fileSize();
        closeFile(); // sync() + close()
        syncVolume();
    }

    // Capture the summary. The clip duration is the number of samples captured
    // (_fifoSeq) times the seconds-per-sample (_tscale), in ms.
    _summary.valid = true;
    _summary.fileName = _startedName + ".gcsv";
    _summary.videoFileName = _videoFileName;
    _summary.durationMs = (uint32_t)((double)_fifoSeq * _tscale * 1000.0);
    _summary.fileSizeBytes = _finalFileSizeBytes;
    // Total space from the volume's cluster count (cheap: read from the FAT boot
    // sector, no FAT walk). We deliberately do NOT call freeClusterCount() here:
    // on a large card it walks the entire FAT (tens of seconds) and would stall
    // the stop path. Free space is refreshed lazily via refreshFreeSpace().
    if(_sd.fatType() != 0)
    {
        uint32_t bpc = _sd.bytesPerCluster();
        _summary.totalBytes = (uint64_t)_sd.clusterCount() * bpc;
        _summary.freeBytes = 0;
    }

    _state = State::Idle;
    DEBUG_INFO("[GYRO] end(): closed '%s', size=%lu bytes, %lu samples, %lu I2C failures",
        _gcsvPath.c_str(), (unsigned long)_finalFileSizeBytes, (unsigned long)_fifoSeq, (unsigned long)_i2cFailures);
    return true;
}

void GyroLogWriter::refreshFreeSpace()
{
    if(_sd.fatType() == 0)
        return; // not mounted

    uint32_t bpc = _sd.bytesPerCluster();
    uint32_t tFree = micros();
    int32_t freeClusters = _sd.freeClusterCount();
    uint32_t freeMs = (uint32_t)((micros() - tFree) / 1000UL);
    if(freeClusters > 0)
        _summary.freeBytes = (uint64_t)freeClusters * bpc;
    DEBUG_INFO("[GYRO] refreshFreeSpace: freeClusters=%ld, took %lu ms",
      (long)freeClusters, (unsigned long)freeMs);
}

void GyroLogWriter::applySlateName(const std::string& slateName, const std::string& extension)
{
    (void)extension;

    // Ignore placeholders / empty names.
    if(slateName.empty() || slateName == "Next Clip" || slateName == "next clip")
        return;

    // Only act on a finalised log (end() has run, file is closed).
    if(_state != State::Idle)
        return;

    std::string safe = sanitiseClipName(slateName);
    if(safe.empty())
        safe = slateName;

    char oldPath[128];
    char newPath[128];
    snprintf(oldPath, sizeof(oldPath), "/%s.gcsv", _startedName.c_str());
    snprintf(newPath, sizeof(newPath), "/%s.gcsv", safe.c_str());

    if(std::strcmp(oldPath, newPath) != 0)
    {
        DEBUG_INFO("[GYRO] applySlateName: rename '%s' -> '%s'", oldPath, newPath);
        if(_sd.rename(oldPath, newPath))
        {
            _gcsvPath = newPath;
            _startedName = safe;
            _videoFileName = safe + "." + _extension;
            syncVolume(); // commit the rename to the card
        }
        else
        {
            DEBUG_INFO("[GYRO] applySlateName: rename FAILED (src exists=%d)", (int)_sd.exists(oldPath));
        }
    }

    // Reflect the new name in the summary shown on the Gyro Log screen.
    _summary.fileName = safe + ".gcsv";
    _summary.videoFileName = _videoFileName;
}

// ---- Orientation (calibration) ----
const char* GyroLogWriter::kNvsNamespace = "GyroLog";
const char* GyroLogWriter::kNvsKeyOrientation = "orient";

void GyroLogWriter::setOrientationIndex(int index)
{
    if(index < 0) index = 0;
    if(index >= kOrientationCount) index = kOrientationCount - 1;
    _orientationIndex = index;
    // Persist to NVS.
    nvs_handle_t handle = 0;
    if(nvs_open(GyroLogWriter::kNvsNamespace, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_u32(handle, GyroLogWriter::kNvsKeyOrientation, (uint32_t)_orientationIndex);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

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

// ---- Filesystem helpers (used by the E2E test) ----
bool GyroLogWriter::fileExists(const std::string& path) const
{
    return _sd.exists(path.c_str());
}

uint64_t GyroLogWriter::fileSize(const std::string& path) const
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return 0;
    uint64_t size = f.fileSize();
    f.close();
    return size;
}

// Count the data rows in a closed GCSV file. A data row is a line whose first
// character is a digit (the "t" sample index). Header lines all start with a
// letter, so this cleanly separates the sample rows from the header.
long GyroLogWriter::countSamplesInFile(const std::string& path)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return -1;

    long count = 0;
    uint8_t buf[512];
    size_t n;
    bool lineStart = true;
    while((n = f.read(buf, sizeof(buf))) > 0)
    {
        for(size_t i = 0; i < n; i++)
        {
            if(lineStart)
            {
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

// Scan a closed GCSV file's "t" index and report its health for Gyroflow: the
// first and last t, the row count, the largest gap between consecutive t values
// (a gap = dropped samples), and whether t ever goes backwards (a reset). A
// clean file has first=0, last=N-1, maxGap=1, and no backwards steps.
void GyroLogWriter::analyzeTIndex(const std::string& path)
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
    {
        DEBUG_INFO("[GYRO] analyzeTIndex('%s'): could not open", path.c_str());
        return;
    }

    uint32_t firstT = 0, lastT = 0, rows = 0, maxGap = 0;
    bool backwards = false;
    bool havePrev = false;
    uint32_t prevT = 0;

    char line[64];
    int c;
    size_t li = 0;
    while((c = f.read()) != -1)
    {
        if(c == '\n')
        {
            line[li] = 0;
            if(li > 0 && line[0] >= '0' && line[0] <= '9')
            {
                // Parse the "t" index (first field, before the first comma).
                uint32_t t = 0;
                size_t k = 0;
                while(k < li && line[k] != ',')
                {
                    t = t * 10 + (uint32_t)(line[k] - '0');
                    k++;
                }
                if(k < li) // had a comma -> it's a data row
                {
                    if(rows == 0)
                        firstT = t;
                    if(havePrev)
                    {
                        if(t < prevT)
                            backwards = true;
                        else
                        {
                            uint32_t gap = t - prevT;
                            if(gap > maxGap)
                                maxGap = gap;
                        }
                    }
                    prevT = t;
                    havePrev = true;
                    lastT = t;
                    rows++;
                }
            }
            li = 0;
        }
        else if(li < sizeof(line) - 1)
        {
            line[li++] = (char)c;
        }
    }
    f.close();

    DEBUG_INFO("[GYRO] analyzeTIndex('%s'): rows=%lu firstT=%lu lastT=%lu maxGap=%lu backwards=%d -> %s",
        path.c_str(), (unsigned long)rows, (unsigned long)firstT, (unsigned long)lastT,
        (unsigned long)maxGap, (int)backwards,
        (rows > 0 && firstT == 0 && lastT == rows - 1 && maxGap <= 1 && !backwards) ? "CLEAN" : "DEGRADED");
}
