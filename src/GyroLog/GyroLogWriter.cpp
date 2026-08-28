#include "GyroLogWriter.h"

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

bool GyroLogWriter::begin(const std::string& clipName, const std::string& extension, const std::string& timecode, const std::string& lensInfo)
{
    (void)clipName; (void)extension; (void)timecode; (void)lensInfo;
    return false;
}

void GyroLogWriter::poll()
{
    // Stub: no sampling yet.
}

bool GyroLogWriter::end()
{
    // Stub: no active log.
    return false;
}

void GyroLogWriter::applySlateName(const std::string& slateName, const std::string& extension)
{
    (void)slateName; (void)extension;
    // Stub: no file to rename.
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
    // Stub: no persistence yet; keep the default (0 = "XYZ").
}
