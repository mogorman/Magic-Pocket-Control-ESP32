#include "GyroLogWriter.h"
#include <SPI.h>
#include <cstring> // strcmp
#include <M5Unified.h> // M5.In_I2C (MPU6886 FIFO self-test)

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

// Mount the SD card via SdFat. The Core2's microSD slot is on the SPI bus
// with its chip-select on GPIO4. We share the SPI bus with the M5GFX display
// (SHARED_SPI), so SdFat toggles only the SD's CS pin. The display already
// initialised the VSPI bus, so we tell SdFat NOT to re-begin it
// (USER_SPI_BEGIN) -- re-initialising an active SPI bus corrupts the MISO
// path. 20 MHz is a safe, verified clock for the card.
bool GyroLogWriter::ensureSd()
{
    // If a valid FAT volume is already mounted, it's ready. fatType() is
    // non-zero only when a real FAT16/32/exFAT volume is mounted, so this
    // doubles as the "card present" check.
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
// pointers) to be written to the card. FatFile::close() flushes the file's
// own data, but the directory entry that records a file's size lives in a
// separate RAM cache that FatFs otherwise only commits on unmount. We close
// the file and then sit idle, so without this the file would show as 0 bytes
// when the card is read on a computer.
void GyroLogWriter::syncVolume()
{
    if(_sd.fatType() == 0)
        return; // not mounted

    _sd.end();
    SPI.begin();
    _sd.begin(SdSpiConfig(4, SHARED_SPI | USER_SPI_BEGIN, SD_SCK_MHZ(20)));
}

// Write the "hello world" payload for a clip to the file at `path`.
bool GyroLogWriter::writeHelloWorld(const std::string& path, const std::string& clipName)
{
    FatFile f;
    if(!f.open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC))
    {
        DEBUG_INFO("[GYRO] writeHelloWorld: open FAILED for '%s' (err=%d)", path.c_str(), (int)f.getError());
        return false;
    }

    char buf[160];
    int n = snprintf(buf, sizeof(buf), "hello world\nclip: %s\n", clipName.c_str());
    if(n < 0) n = 0;
    if(f.write((const uint8_t*)buf, (size_t)n) != n)
    {
        DEBUG_INFO("[GYRO] writeHelloWorld: write FAILED for '%s'", path.c_str());
        f.close();
        return false;
    }

    f.close();
    return true;
}

bool GyroLogWriter::begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo)
{
    (void)lensInfo;

    if(_state == State::Recording)
        return false; // already recording

    // Make sure the SD card is mounted before we try to open the file.
    if(!ensureSd())
    {
        DEBUG_INFO("[GYRO] begin(): SD not ready (msg='%s')", _sdStatusMessage.c_str());
        return false;
    }

    _startedName = clipName;
    _extension = extension;
    _gcsvPath = "/" + clipName + ".txt";

#if GYRO_MOCK_DATA
    // Open the data file and write the GCSV header. The mock sampler (poll())
    // then appends one fixed-width row per 1 ms tick, so the finished file size
    // is exactly predictable: header + rows * kMockRowBytes.
    _file.close();
    if(!_file.open(_gcsvPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC))
    {
        DEBUG_INFO("[GYRO] begin(): open FAILED for '%s' (err=%d)", _gcsvPath.c_str(), (int)_file.getError());
        return false;
    }

    int headerBytes = writeGcsvHeader(timecode, _startedName + "." + _extension);
    if(headerBytes < 0)
    {
        DEBUG_INFO("[GYRO] begin(): failed to write header to '%s'", _gcsvPath.c_str());
        _file.close();
        return false;
    }

    _mockSeq = 0;
    _mockLastTick = millis();
    _mockRowsWritten = 0;
    _mockHeaderBytes = (uint32_t)headerBytes;

    DEBUG_INFO("[GYRO] begin(): opened '%s', wrote %d-byte header (clip='%s')",
        _gcsvPath.c_str(), headerBytes, clipName.c_str());
#else
    // No mock data: just create the file with the "hello world" payload.
    if(!writeHelloWorld(_gcsvPath, clipName))
    {
        DEBUG_INFO("[GYRO] begin(): failed to create '%s'", _gcsvPath.c_str());
        return false;
    }
    DEBUG_INFO("[GYRO] begin(): wrote '%s' (clip='%s')", _gcsvPath.c_str(), clipName.c_str());
#endif

    _state = State::Recording;
    return true;
}

void GyroLogWriter::poll()
{
#if GYRO_MOCK_DATA
    if(_state != State::Recording)
        return;
    if(!_file.isOpen())
        return;

    // Write one row per ~1 ms tick. We gate on the millisecond changing so the
    // row rate tracks the real 1 kHz sampler even though the main loop runs at an
    // irregular cadence (it also does UI work). A tick that the loop skips is
    // simply a gap in the "t" index -- the same as a dropped real sample.
    uint32_t now = millis();
    if(now == _mockLastTick)
        return; // not a new millisecond yet
    _mockLastTick = now;

    // Fixed-width mock row: "t,gx,gy,gz,ax,ay,az\n". The "t" index is
    // zero-padded to a fixed width so every row is exactly kMockRowBytes, which
    // makes the finished file size exactly predictable. Values are a simple
    // pattern (seq) so a reader can confirm the sequence is intact.
    char row[48];
    int n = snprintf(row, sizeof(row), "%06lu,0,0,0,0,0,0\n", (unsigned long)_mockSeq);
    if(n < 0)
        n = 0;
    if(n != (int)kMockRowBytes)
    {
        // The row width is a build-time invariant; if it ever changes, log once
        // so the size prediction is not silently wrong.
        DEBUG_ERROR("[GYRO] poll(): mock row width %d != expected %d", n, (int)kMockRowBytes);
    }

    if(_file.write((const uint8_t*)row, (size_t)n) != n)
    {
        DEBUG_INFO("[GYRO] poll(): SD write FAILED at row %lu (err=%d)",
            (unsigned long)_mockSeq, (int)_file.getError());
        return;
    }

    _mockSeq++;
    _mockRowsWritten++;
#else
    // No mock data (data side not ported yet).
    (void)0;
#endif
}

bool GyroLogWriter::end()
{
    if(_state != State::Recording)
        return false;

    _state = State::Idle;

    // Close the file (flushes any buffer) and commit the directory entry to the
    // card so it's visible on a computer.
    if(_file.isOpen())
    {
        _finalFileSize = _file.fileSize();
        _file.close();
    }
    syncVolume();

    // Capture the summary for the Gyro Log screen.
    _summary.valid = true;
    _summary.fileName = _startedName + ".txt";
    _summary.videoFileName = _startedName + "." + _extension;
    _summary.fileSizeBytes = _finalFileSize;
#if GYRO_MOCK_DATA
    _summary.mockRows = _mockRowsWritten;
    _summary.mockHeaderBytes = _mockHeaderBytes;
#endif
    // Total space comes from the boot sector (cluster count x
    // bytes-per-cluster) -- cheap, so do it here. Free space is NOT computed
    // here: freeClusterCount() walks the whole FAT (tens of seconds on a large
    // exFAT/FAT32 card) and would stall the stop path. It is refreshed lazily
    // via refreshFreeSpace() from a non-critical context instead.
    if(_sd.fatType() != 0)
    {
        uint32_t bpc = _sd.bytesPerCluster();
        _summary.totalBytes = (uint64_t)_sd.clusterCount() * bpc;
        _summary.freeBytes = 0;
    }

#if GYRO_MOCK_DATA
    DEBUG_INFO("[GYRO] end(): closed '%s', size=%lu bytes, %lu mock rows, %lu-byte header",
        _gcsvPath.c_str(), (unsigned long)_finalFileSize,
        (unsigned long)_mockRowsWritten, (unsigned long)_mockHeaderBytes);
#else
    DEBUG_INFO("[GYRO] end(): closed '%s', size=%lu bytes", _gcsvPath.c_str(), (unsigned long)_finalFileSize);
#endif
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

    char oldPath[128];
    char newPath[128];
    snprintf(oldPath, sizeof(oldPath), "/%s.txt", _startedName.c_str());
    snprintf(newPath, sizeof(newPath), "/%s.txt", slateName.c_str());

    if(std::strcmp(oldPath, newPath) != 0)
    {
        DEBUG_INFO("[GYRO] applySlateName: rename '%s' -> '%s'", oldPath, newPath);
        if(_sd.rename(oldPath, newPath))
        {
            _gcsvPath = newPath;
            _startedName = slateName;
            syncVolume(); // commit the rename to the card
        }
        else
        {
            DEBUG_INFO("[GYRO] applySlateName: rename FAILED (src exists=%d)", (int)_sd.exists(oldPath));
        }
    }

    // In mock mode the file holds real GCSV data (header + rows); we must NOT
    // clobber it with the "hello world" payload, so we only rename it. In
    // non-mock mode the file is the hello-world stub, which we rewrite so its
    // contents match the real clip name.
#if !GYRO_MOCK_DATA
    if(writeHelloWorld(_gcsvPath, slateName))
        syncVolume();
#endif

    // Reflect the new name in the summary shown on the Gyro Log screen.
    _summary.fileName = slateName + ".txt";
}

void GyroLogWriter::setOrientationIndex(int index)
{
    if(index < 0) index = 0;
    if(index >= kOrientationCount) index = kOrientationCount - 1;
    _orientationIndex = index;
}

const char* GyroLogWriter::orientationToken(int index)
{
    if(index < 0) index = 0;
    if(index >= kOrientationCount) index = kOrientationCount - 1;
    return GYROLOG_ORIENTATION_TOKENS[index];
}

void GyroLogWriter::loadOrientation()
{
    // No persistence yet; keep the default (0 = "XYZ").
}

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

// ---- MPU6886 FIFO self-test ----
// The MPU6886 sits at I2C address 0x68 on the Core2's internal I2C bus
// (M5.In_I2C), which M5Unified already initialised. We talk to it directly at
// 400 kHz (the same rate the original branch used).
static const uint8_t kImuAddr = 0x68;
static const uint32_t kImuI2cHz = 400000;

bool GyroLogWriter::configureFifo(int smplrtDiv)
{
    // Wake the sensor and select the clock source. Per the MPU-6886 datasheet
    // (DS-000193 v1.1) section 9.6, CLKSEL[2:0] MUST be 001 (auto-select: PLL
    // gyro clock if ready, else internal oscillator) for full performance. The
    // value 0x01 sets CLKSEL=001 and leaves SLEEP/CYCLE/GYRO_STANDBY/TEMP_DIS
    // all clear. (An earlier attempt used 0x40, which is GYRO_STANDBY with
    // CLKSEL=000 = internal 20 MHz oscillator only -- that leaves the sensor's
    // data path not running, so the FIFO never fills.)
    M5.In_I2C.writeRegister8(kImuAddr, 0x6B, 0x01, kImuI2cHz);
    delay(10); // let the PLL lock

    // Sample-rate divider. 0 = no divider = the sensor's native rate.
    M5.In_I2C.writeRegister8(kImuAddr, 0x19, (uint8_t)smplrtDiv, kImuI2cHz);

    // DLPF. CONFIG = 0x01 (44 Hz DLPF) is the highest setting where the sample
    // rate divider still works; keep it as M5Unified set it.
    M5.In_I2C.writeRegister8(kImuAddr, 0x1A, 0x01, kImuI2cHz);

    // Enable the FIFO for gyro + accel (10 bytes per packet).
    M5.In_I2C.writeRegister8(kImuAddr, 0x23, 0x08, kImuI2cHz); // FIFO_EN: GYRO|ACCEL

    // USER_CTRL: FIFO_EN (bit6) + FIFO_RST (bit5) to enable and clear the FIFO.
    M5.In_I2C.writeRegister8(kImuAddr, 0x6A, 0x40, kImuI2cHz);
    delay(10);

    // Read back the key registers to confirm the FIFO is really enabled.
    uint8_t who = M5.In_I2C.readRegister8(kImuAddr, 0x75, kImuI2cHz);
    uint8_t pwr = M5.In_I2C.readRegister8(kImuAddr, 0x6B, kImuI2cHz);
    uint8_t smpl = M5.In_I2C.readRegister8(kImuAddr, 0x19, kImuI2cHz);
    uint8_t cfg = M5.In_I2C.readRegister8(kImuAddr, 0x1A, kImuI2cHz);
    uint8_t fifoEn = M5.In_I2C.readRegister8(kImuAddr, 0x23, kImuI2cHz);
    uint8_t user = M5.In_I2C.readRegister8(kImuAddr, 0x6A, kImuI2cHz);
    uint8_t cntHi = M5.In_I2C.readRegister8(kImuAddr, 0x72, kImuI2cHz);
    uint8_t cntLo = M5.In_I2C.readRegister8(kImuAddr, 0x73, kImuI2cHz);
    DEBUG_INFO("[GYRO] configureFifo readback: WHO_AM_I=0x%02X PWR=0x%02X (CLKSEL=%d) SMPL=0x%02X CFG=0x%02X FIFO_EN=0x%02X USER=0x%02X FIFO_COUNT=%u",
        who, pwr, (pwr & 0x07), smpl, cfg, fifoEn, user, (cntHi << 8) | cntLo);

    // Also read raw output-register samples to confirm the sensor is producing
    // changing data (independent of the FIFO). Read gyro (0x43) five times,
    // 100 ms apart, to see if the values move at all.
    uint8_t rawG[6];
    for(int i = 1; i <= 5; i++)
    {
        M5.In_I2C.readRegister(kImuAddr, 0x43, rawG, 6, kImuI2cHz);
        DEBUG_INFO("[GYRO] gyro sample #%d: gx=%d gy=%d gz=%d",
            i,
            (int16_t)((rawG[0] << 8) | rawG[1]), (int16_t)((rawG[2] << 8) | rawG[3]), (int16_t)((rawG[4] << 8) | rawG[5]));
        if(i < 5)
            delay(100);
    }
    DEBUG_INFO("[GYRO] configureFifo: FIFO enabled (gyro+accel), SMPLRT_DIV=%d", smplrtDiv);
    return true;
}

float GyroLogWriter::measureFifoRate()
{
    // The FIFO holds up to 1024 samples. If the sensor's native rate is ~3.8
    // kHz, the FIFO fills in ~270 ms and overflows (new samples are dropped).
    // We drain it as fast as the I2C bus allows and count packets, so the
    // measured rate reflects what the bus can actually sustain.

    uint32_t t0 = micros();
    uint32_t packets = 0;
    uint32_t maxCount = 0;
    int firstGx = 0, firstGy = 0, firstGz = 0;
    bool gotFirst = false;

    // A 10-byte packet: accel X/Y/Z, temp, gyro X/Y/Z (big-endian).
    uint8_t pkt[10];

    for(;;)
    {
        uint32_t now = micros();
        if(now - t0 >= 3000000) // 3 s
            break;

        // Read the FIFO count (upper 9 bits, in 2-byte units) from 0x72:0x73.
        uint8_t ch = M5.In_I2C.readRegister8(kImuAddr, 0x72, kImuI2cHz);
        uint8_t cl = M5.In_I2C.readRegister8(kImuAddr, 0x73, kImuI2cHz);
        uint16_t count = ((uint16_t)ch << 8) | cl; // number of 2-byte units

        if(count == 0)
            continue;

        if(count > maxCount)
            maxCount = count;

        // Drain the FIFO. Read in 10-byte packets. (We read a little more than
        // is present on the last one; the sensor wraps, but for a rate
        // measurement over 3 s that's negligible.)
        uint16_t packetsToRead = count / 2; // 10-byte packets
        for(uint16_t i = 0; i < packetsToRead; i++)
        {
            if(!M5.In_I2C.readRegister(kImuAddr, 0x3B, pkt, 10, kImuI2cHz))
                break;
            packets++;
            if(!gotFirst)
            {
                firstGx = (int16_t)((pkt[4] << 8) | pkt[5]);
                firstGy = (int16_t)((pkt[6] << 8) | pkt[7]);
                firstGz = (int16_t)((pkt[8] << 8) | pkt[9]);
                gotFirst = true;
            }
        }
    }

    uint32_t elapsedUs = micros() - t0;
    float rate = (float)packets / ((float)elapsedUs / 1000000.0f);

    DEBUG_INFO("[GYRO] FIFO rate: %lu packets in %lu ms = %.1f Hz (max FIFO count seen: %lu 2-byte units)",
        (unsigned long)packets, (unsigned long)(elapsedUs / 1000), rate, (unsigned long)maxCount);
    if(gotFirst)
        DEBUG_INFO("[GYRO] FIFO first sample: gx=%d gy=%d gz=%d", firstGx, firstGy, firstGz);

    return rate;
}

#if GYRO_MOCK_DATA
// Write the GCSV header block to the already-open data file. The header is a
// fixed set of "key,value" lines (the GYROFLOW IMU LOG format). The "note" and
// "videofilename" fields embed the clip name, so the header length depends on
// the name length -- but that is fine: the E2E test reads the actual file size
// after the fact rather than predicting it, and the data rows (the part that
// must be exactly predictable) are fixed-width. Returns the number of bytes
// written, or -1 on failure.
int GyroLogWriter::writeGcsvHeader(const std::string& timecode, const std::string& videoFileName)
{
    char header[512];
    int n = snprintf(header, sizeof(header),
        "GYROFLOW IMU LOG\n"
        "version,1.3\n"
        "id,m5stack-core2-mpu6886\n"
        "orientation,%s\n"
        "note,M5Stack Core2 gyro log ~1kHz (mock); start TC %s\n"
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
        videoFileName.c_str(),
        0.001, // tscale: 1 ms per sample (1 kHz)
        0.000172685, // gscale (deg/s -> rad/s, +/-2000 deg/s)
        0.000244141); // ascale (g -> m/s^2, +/-8 g)

    if(n < 0)
        return -1;
    if(_file.write((const uint8_t*)header, (size_t)n) != n)
        return -1;
    return n;
}

// Re-open the file read-only and confirm the body is complete and well-formed:
// exactly `expectedRows` data rows, the last row's "t" index == expectedRows-1
// (no dropped rows), and the file ends with a newline. This is the "written
// completely" check the E2E test needs on top of the size check.
bool GyroLogWriter::verifyFileComplete(const std::string& path, uint32_t expectedRows) const
{
    FatFile f;
    if(!f.open(path.c_str(), O_RDONLY))
        return false;

    // Read the whole file (a few hundred KB at most for a short clip) and scan
    // it. We count data rows (lines that are not the header block) and track
    // the last "t" index we saw.
    uint32_t rows = 0;
    uint32_t lastT = 0;
    bool sawHeader = false;
    bool endsWithNewline = false;
    char line[64];

    // First, note whether the file ends with a newline.
    uint64_t size = f.fileSize();
    if(size > 0)
    {
        f.seekSet(size - 1);
        char last = 0;
        if(f.read(&last, 1) == 1)
            endsWithNewline = (last == '\n');
    }
    f.seekSet(0);

    // Line-by-line scan. We read char-by-char into a small buffer; for a
    // short test clip this is fine and avoids pulling in a line-reading helper.
    int c;
    size_t li = 0;
    while((c = f.read()) != -1)
    {
        if(c == '\n')
        {
            line[li] = 0;
            if(li > 0)
            {
                // A data row looks like "NNNNNN,...". The header lines are
                // "key,value" and the column header "t,gx,..." -- none of those
                // start with a 6-digit number, so we can tell them apart by the
                // first field.
                if(line[0] >= '0' && line[0] <= '9' && li >= 6)
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
                        rows++;
                        lastT = t;
                    }
                }
                else if(strncmp(line, "GYROFLOW IMU LOG", 16) == 0)
                {
                    sawHeader = true;
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

    bool ok = sawHeader && (rows == expectedRows) && (lastT == expectedRows - 1) && endsWithNewline;
    DEBUG_INFO("[GYRO] verifyFileComplete('%s'): rows=%lu expected=%lu lastT=%lu expectedLastT=%lu endsNL=%d -> %s",
        path.c_str(), (unsigned long)rows, (unsigned long)expectedRows,
        (unsigned long)lastT, (unsigned long)(expectedRows - 1), (int)endsWithNewline, ok ? "OK" : "MISMATCH");
    return ok;
}
#endif // GYRO_MOCK_DATA
