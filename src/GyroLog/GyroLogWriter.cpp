#include "GyroLogWriter.h"
#include <SPI.h>
#include <cstring> // strcmp

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
    (void)timecode;
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

    // Create the file with the "hello world" payload.
    if(!writeHelloWorld(_gcsvPath, clipName))
    {
        DEBUG_INFO("[GYRO] begin(): failed to create '%s'", _gcsvPath.c_str());
        return false;
    }

    DEBUG_INFO("[GYRO] begin(): wrote '%s' (clip='%s')", _gcsvPath.c_str(), clipName.c_str());

    _state = State::Recording;
    return true;
}

void GyroLogWriter::poll()
{
    // No sampling yet (data side only for now).
}

bool GyroLogWriter::end()
{
    if(_state != State::Recording)
        return false;

    _state = State::Idle;

    // The file was fully written in begin(); close it (flushes any buffer) and
    // commit the directory entry to the card so it's visible on a computer.
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
    // Total space comes from the FAT boot sector (cluster count x
    // bytes-per-cluster) -- cheap, so do it here. Free space is NOT computed
    // here: freeClusterCount() walks the whole FAT (tens of seconds on FAT32)
    // and would stall the stop path. It is refreshed lazily via
    // refreshFreeSpace() from a non-critical context instead.
    if(_sd.fatType() != 0)
    {
        uint32_t bpc = _sd.bytesPerCluster();
        _summary.totalBytes = (uint64_t)_sd.clusterCount() * bpc;
        _summary.freeBytes = 0;
    }

    DEBUG_INFO("[GYRO] end(): closed '%s', size=%lu bytes", _gcsvPath.c_str(), (unsigned long)_finalFileSize);
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

    // Rewrite the file contents to match the real clip name.
    if(writeHelloWorld(_gcsvPath, slateName))
        syncVolume();

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
