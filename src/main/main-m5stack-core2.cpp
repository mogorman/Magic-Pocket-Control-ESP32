#define USING_TFT_ESPI 0          // Not using the TFT_eSPI graphics library <-- must include this in every main file, 0 = not using, 1 = using
#define USING_M5GFX 0             // Using the M5GFX library with touch screen
#define USING_M5_BUTTONS 1        // Using the M5GFX library with the 3 buttons (buttons A, B, C)

#define OUTPUT_CAMERA_SETTINGS 1  // 1 = Outputs camera settings through serial (so other applications can read them)

// End-to-end self-test. Set GYRO_E2E_TEST to 1 to run it (default 0 = compiled
// out for a clean production build). It boots into the real app, then drives
// the production record path exactly as the record button would:
//   1. after a short settle delay, queue a record START (gyroPendingStart ->
//      the main loop calls gyroLog.begin(), which writes the hello-world file);
//   2. after GYRO_E2E_DURATION_S seconds, queue a record STOP (gyroPendingEnd ->
//      the main loop calls gyroLog.end(), which closes + commits the file);
//   3. once end() has closed the file, read it back and verify it exists and is
//      the expected size, then report a PASS/FAIL.
// This exercises the full production SD path (begin/end/syncVolume) and checks
// the resulting file is well-formed, without needing a camera connected.
#ifndef GYRO_E2E_TEST
#define GYRO_E2E_TEST 1
#endif
// How long (seconds) the E2E test records before stopping.
#ifndef GYRO_E2E_DURATION_S
#define GYRO_E2E_DURATION_S 10
#endif
// How long (ms) to wait AFTER the camera is connected before the E2E test
// triggers the record start, so the connection/SD code has a moment to settle.
#ifndef GYRO_E2E_SETTLE_MS
#define GYRO_E2E_SETTLE_MS 3000
#endif

// FIFO self-test. Set to 1 to run a one-shot MPU6886 FIFO rate measurement at
// boot (enables the gyro+accel FIFO, drains it for ~3 s, and logs the measured
// packets/second). Default 0 = compiled out.
#ifndef GYRO_FIFO_TEST
#define GYRO_FIFO_TEST 1
#endif

// The output format is: >>[state]:[state value]
// Here are some examples:
/*
>>LensType:Canon EF-S 18-55mm f/3.5-5.6
>>LensIris:f3.5
>>FocalLengthMM:18mm
>>LensDistance:460mm to 590mm
>>ISO:3200
>>WhiteBalance:6500
>>Tint:10
>>Aperture:f3.5
>>ApertureNormalised:0
>>HasLens:Yes
>>FocalLengthMM:18
>>ModelName:Pocket Cinema Camera 6K
>>IsPocketCamera:Yes
>>FrameRate:25
>>FrameDims:4096 x 2160
>>FrameSize:4K DCI
>>ShutterAngle:6000
>>Codec:BRAW 12:1
*/

#include <M5Unified.h>

#define tft M5.Lcd
static LGFX_Sprite *sprite;

#include <Arduino.h>
#include <string.h>

#include "Arduino_DebugUtils.h" // Debugging to Serial - https://github.com/arduino-libraries/Arduino_DebugUtils

// Main BMD Libraries
#include "Camera/ConstantsTypes.h"
#include "Camera/PacketWriter.h"
#include "CCU/CCUUtility.h"
#include "CCU/CCUPacketTypes.h"
#include "CCU/CCUValidationFunctions.h"
#include "Camera/BMDCameraConnection.h"
#include "Camera/BMDCamera.h"
#include "BMDControlSystem.h"
#include "GyroLog/GyroLogWriter.h"

// Include the watchdog library so we can stop it timing out while pass key entry.
#include "esp_task_wdt.h"

// Lato font from Google Fonts
// Agency FB font is free for commercial use, copied from Windows fonts
// Rather than using font sizes, we use specific fonts for each size as it renders better on screen
#include "Fonts/Lato_Regular12pt7b.h" // Slightly larger version
#include "Fonts/Lato_Regular11pt7b.h" // Standard font
#include "Fonts/Lato_Regular6pt7b.h" // Small version for camera address
#include "Fonts/Lato_Regular5pt7b.h" // Smallest version
#include "Fonts/AgencyFB_Regular7pt7b.h" // Agency FB for tiny text
#include "Fonts/AgencyFB_Bold9pt7b.h" // Agency FB small for above buttons
#include "Fonts/AgencyFB_Regular9pt7b.h" // Agency FB small-medium for above buttons

// Screen width and height
#define IWIDTH 320
#define IHEIGHT 240

// Sprite width and height (limited by the lack of PSRAM on this device, Set to 8bpp, instead of 16bpp)
// We'll draw the side bar, the button labels, and the recording rectangle separately to the sprite
// Note: If you try to use a sprite that takes too much memory it may not show at all or Bluetooth will not connect properly
#define IWIDTH_SPRITE 320 // 300
#define IHEIGHT_SPRITE 240 // 208
#define BPP_SPRITE 8

// Sprites for Images
// Not using sprites to hold images as there's no PSRAM to store them and the main content sprite, so we'll just load them from other memory when required
/*
LGFX_Sprite spriteMPCSplash;
LGFX_Sprite spriteBluetooth;
LGFX_Sprite spritePocket4k;
LGFX_Sprite spriteWBBright;
LGFX_Sprite spriteWBCloud;
LGFX_Sprite spriteWBFlourescent;
LGFX_Sprite spriteWBIncandescent;
LGFX_Sprite spriteWBMixedLight;
LGFX_Sprite spriteWBBrightBG;
LGFX_Sprite spriteWBCloudBG;
LGFX_Sprite spriteWBFlourescentBG;
LGFX_Sprite spriteWBIncandescentBG;
LGFX_Sprite spriteWBMixedLightBG;
*/

// Images
#include "Images/MPCSplash-M5Stack-CoreS3.h"
#include "Images/ImageBluetooth.h"
#include "Images/ImagePocket4k.h"
#include "Images/WBBright.h"
#include "Images/WBCloud.h"
#include "Images/WBFlourescent.h"
#include "Images/WBIncandescent.h"
#include "Images/WBMixedLight.h"
#include "Images/WBBrightBG.h"
#include "Images/WBCloudBG.h"
#include "Images/WBFlourescentBG.h"
#include "Images/WBIncandescentBG.h"
#include "Images/WBMixedLightBG.h"

BMDCameraConnection cameraConnection;
std::shared_ptr<BMDControlSystem> BMDControlSystem::instance = nullptr; // Required for Singleton pattern and the constructor for BMDControlSystem
BMDCameraConnection* BMDCameraConnection::instancePtr = &cameraConnection; // Required for the scan function to run non-blocking and call the object back with the result

enum class Screens : byte
{
  PassKey = 9,
  NoConnection = 10,
  Dashboard = 100,
  Recording = 101,
  ISO = 102,
  ShutterAngleSpeed = 103,
  WhiteBalanceTintWB = 104, // Editing White Balance
  WhiteBalanceTintT = 124, // Editing Tint
  Codec = 105,
  Framerate = 106,
  Resolution = 107,
  Media = 108,
  Lens = 109,
  Slate = 110,
  Project = 111,
  GyroLog = 112
};

Screens connectedScreenIndex = Screens::NoConnection; // The index of the screen we're on:
// 9 is Pass Key
// 10 is No Connection
// 100 is Dashboard
// 101 is Recording
// 102 is ISO
// 103 is Shutter Angle & Shutter Speed
// 104 is WB / Tint - Edit WB
// 105 is Codec
// 106 is Framerate
// 107 is Resolution (one for each camera group, 4K, 6K/G2/Pro, Mini Pro G2, Mini Pro 12K)
// 108 is Media
// 109 is Lens
// 112 is Gyro Log (default page when connected)
// 124 is WB / Tint - Edit Tint

// Keep track of the last camera modified time that we refreshed a screen so we don't keep refreshing a screen when the camera object remains unchanged.
static unsigned long lastRefreshedScreen = 0;

// Button pressed indications for use in each individual page
bool btnAPressed = false;
bool btnBPressed = false;

// To learn the real name of the clip that was just recorded, we briefly switch
// the camera into PLAYBACK mode right after a clip stops. The camera then
// pushes the clip's file name (e.g. "A010_08260408_C022") over CCU, which we
// use to display on the Gyro Log screen. Once the name is received we switch
// the camera back to its previous (preview) state.
//
// The clip-name capture is a single boolean, not a multi-state machine:
//   * On a genuine record stop we flip the camera into PLAYBACK and set
//     gyroInPlayback to true.
//   * While gyroInPlayback is true we wait for the camera to push the real
//     clip name (it does so repeatedly while in playback). The first real name
//     is stored and we flip the camera back to PREVIEW and clear the flag.
//   * A 6-second timeout does the same in case no name arrives.
//   * The record start/stop callbacks ignore everything while gyroInPlayback is
//     set, so the playback/preview toggling we do ourselves can't re-trigger
//     them. The next genuine record stop simply sets the flag again.
//
// All the actual BLE writes happen in the main loop (never the notify
// handler, where a blocking write would deadlock).
static bool gyroInPlayback = false;

// The real clip name (file name without extension) received from the camera
// during playback, queued for the main loop to display.
static std::string gyroPendingSlateName;
static bool gyroPendingSlateNameValid = false;

// A queued "start a new gyro log" request, set by the record-start callback
// and applied by the main loop (begin() does SD work and must not run on the
// BLE notify thread).
struct GyroPendingStart
{
  std::string clipName;
  std::string ext;
  std::string timecode;
  bool valid = false;
};
static GyroPendingStart gyroPendingStart;

// Set by the record-stop callback to ask the main loop to finalise the log
// (end() does a blocking SD unmount/remount and must not run on the BLE
// notify thread while the main loop may be touching the same card).
static bool gyroPendingEnd = false;

// Set synchronously in the record-start callback the moment a start is queued,
// and cleared by the main loop once begin() has run (or failed). The camera can
// push the record state twice in a row before the main loop has processed the
// first, so this flag (set on the notify thread, not the main loop) is what
// de-duplicates a spurious second start.
static bool gyroStartArmed = false;

// Set (on the BLE notify thread) once, after a clip has stopped and the camera
// has been flipped back, to ask the main loop to compute the SD free-space
// figure. freeClusterCount() walks the whole FAT (~tens of seconds) and must not
// run on the main loop, so we run it here on the notify thread instead, where a
// long block is harmless. The result is cached in the volume, so the Gyro Log
// screen can then read summary.freeBytes cheaply.
static bool gyroFreeSpacePending = false;

// The codec in effect when the clip started, so we can build the right video
// file extension once we have the real clip name.
static CCUPacketTypes::BasicCodec gyroStartCodec = CCUPacketTypes::BasicCodec::BRAW;

// Gyro log writer for the Core2's SD card. (Data side only for now: writes a
// "hello world" text file per clip; the 1 kHz GCSV sampling is a later step.)
GyroLogWriter gyroLog;

// The generic clip-name counter used when the camera hasn't sent a slate name
// yet (e.g. "clip_0001"). Kept in RAM for now (NVS persistence is a later
// step).
static uint32_t gyroClipCounter = 0;

// Build the next generic clip name ("clip_0001", "clip_0002", ...).
static std::string nextGyroClipName()
{
  gyroClipCounter++;
  char buf[32];
  snprintf(buf, sizeof(buf), "clip_%04lu", (unsigned long)gyroClipCounter);
  return std::string(buf);
}

// Map the camera's current codec to the video file extension we'd pair the
// gyro log with.
static std::string gyroVideoExtension(BMDCamera* cam)
{
  if(cam && cam->hasCodec())
  {
    switch(cam->getCodec().basicCodec)
    {
      case CCUPacketTypes::BasicCodec::ProRes: return "mov";
      case CCUPacketTypes::BasicCodec::RAW:    return "raw";
      case CCUPacketTypes::BasicCodec::DNxHD:  return "mxf";
      case CCUPacketTypes::BasicCodec::BRAW:   return "braw";
      default: break;
    }
  }
  return "braw";
}

// Display elements on the screen common to all pages
void Screen_Common(int sideBarColour)
{
    // DEBUG_DEBUG("Screen_Common");

    // Sidebar colour
    sprite->fillRect(0, 0, 13, IHEIGHT, sideBarColour);
    sprite->fillRect(13, 0, 2, IHEIGHT, TFT_DARKGREY);

    if(BMDControlSystem::getInstance()->hasCamera())
    {
      auto camera = BMDControlSystem::getInstance()->getCamera();

      sprite->setTextColor(TFT_WHITE);

      if(connectedScreenIndex == Screens::Dashboard)
      {
        // Dashboard Bottom Buttons
        sprite->fillSmoothRoundRect(30, 210, 80, 40, 3, TFT_DARKCYAN);
        sprite->drawCenterString("TBD", 70, 217, &AgencyFB_Bold9pt7b);

        if(camera->isRecording)
          sprite->fillSmoothCircle(IWIDTH / 2, 240, 30, TFT_RED);
        else
        {
          // Two outlines to make it a bit thicker
          sprite->drawCircle(IWIDTH / 2, 240, 30, TFT_RED);
          sprite->drawCircle(IWIDTH / 2, 240, 29, TFT_RED);
        }
      }
      else if(connectedScreenIndex == Screens::Recording)
      {
        // Recording Screen Bottom Buttons
        sprite->fillSmoothRoundRect(30, 210, 80, 40, 3, TFT_DARKCYAN);
        sprite->drawCenterString("TBD", 70, 217, &AgencyFB_Bold9pt7b);

        if(camera->isRecording)
          sprite->fillSmoothCircle(IWIDTH / 2, 240, 30, TFT_RED);
        else
        {
          // Two outlines to make it a bit thicker
          sprite->drawCircle(IWIDTH / 2, 240, 30, TFT_RED);
          sprite->drawCircle(IWIDTH / 2, 240, 29, TFT_RED);
        }
      }
      else
      {
        // Other Screens Bottom Buttons

        switch(connectedScreenIndex)
        {
          case Screens::ISO:
          case Screens::ShutterAngleSpeed:
          case Screens::WhiteBalanceTintT:
          case Screens::Resolution:
          case Screens::Framerate:
          case Screens::Media:
            sprite->fillSmoothRoundRect(30, 210, 170, 40, 3, TFT_DARKCYAN);
            sprite->fillTriangle(60, 235, 70, 215, 50, 215, TFT_WHITE); // Up Arrow
            sprite->fillTriangle(150, 235, 170, 235, 160, 215, TFT_WHITE); // Down Arrow
            break;
          case Screens::Codec:
            // White Balance shows Presets or increment
            sprite->fillSmoothRoundRect(30, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("CODEC", 70, 217, &AgencyFB_Bold9pt7b);

            sprite->fillSmoothRoundRect(120, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("SETTING", 160, 217, &AgencyFB_Bold9pt7b);
            break;
          case Screens::WhiteBalanceTintWB:
            // White Balance shows Presets or increment
            sprite->fillSmoothRoundRect(30, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("PRESET", 70, 217, &AgencyFB_Bold9pt7b);

            sprite->fillSmoothRoundRect(120, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("+100", 160, 217, &AgencyFB_Bold9pt7b);
            break;
          case Screens::Lens:
            sprite->fillSmoothRoundRect(30, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("FOCUS", 70, 217, &AgencyFB_Bold9pt7b);
            break;
          case Screens::GyroLog:
            // A = previous orientation, B = next orientation (calibration),
            // C = next screen.
            sprite->fillSmoothRoundRect(30, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("< ORIENT", 70, 217, &AgencyFB_Bold9pt7b);

            sprite->fillSmoothRoundRect(120, 210, 80, 40, 3, TFT_DARKCYAN);
            sprite->drawCenterString("ORIENT >", 160, 217, &AgencyFB_Bold9pt7b);
            break;
        }
      }

      // Common Next Button
      sprite->fillSmoothRoundRect(215, 210, 80, 40, 3, TFT_DARKCYAN);
      sprite->drawCenterString("NEXT", 255, 217, &AgencyFB_Bold9pt7b);
    }
}

void Screen_Common_Connected()
{
  if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::Connected && BMDControlSystem::getInstance()->hasCamera())
  {
    Screen_Common(cameraConnection.getInitialPayloadTime() < millis() ? TFT_GREEN : TFT_DARKGREY);

    // Show the recording outline
    if(BMDControlSystem::getInstance()->getCamera()->isRecording)
    {
      // Turn on recording box
      sprite->drawRect(15, 0, IWIDTH - 15, IHEIGHT, TFT_RED);
      sprite->drawRect(16, 1, IWIDTH - 13, IHEIGHT - 2, TFT_RED);
    }
  }
}

// Screen for when there's no connection, it's scanning, and it's trying to connect.
void Screen_NoConnection()
{
  DEBUG_DEBUG("Screen_NoConnection");

  if(!sprite->createSprite(IWIDTH_SPRITE, IHEIGHT_SPRITE)) return;

  // The camera to connect to.
  int connectToCameraIndex = -1;

  connectedScreenIndex = Screens::NoConnection;

  // Background on the sprite (overlay the part of the background that covers the sprite)
  sprite->pushImage(0, 0, IWIDTH, IHEIGHT, MPCSplash_M5Stack_CoreS3);

  // Black background for text and Bluetooth Logo
  sprite->fillRect(0, 3, IWIDTH, 51, TFT_BLACK);

  // Bluetooth Image
  sprite->pushImage(26, 6, 30, 46, Wikipedia_Bluetooth_30x46);

  switch(cameraConnection.status)
  {
    case BMDCameraConnection::Scanning:
      Screen_Common(TFT_BLUE); // Common elements
      sprite->drawString("Scanning...", 70, 20);
      break;
    case BMDCameraConnection::ScanningFound:
      Screen_Common(TFT_BLUE); // Common elements
      if(cameraConnection.cameraAddresses.size() == 1)
      {
        sprite->drawString("Found, connecting...", 70, 20);
        connectToCameraIndex = 0;
      }
      else
        sprite->drawString("Found cameras", 70, 20); // Multiple camera selection is below
      break;
    case BMDCameraConnection::ScanningNoneFound:
      Screen_Common(TFT_RED); // Common elements
      sprite->drawString("No camera found", 70, 20);
      break;
    case BMDCameraConnection::Connecting:
      Screen_Common(TFT_YELLOW); // Common elements
      sprite->drawString("Connecting...", 70, 20);
      break;
    case BMDCameraConnection::NeedPassKey:
      Screen_Common(TFT_PURPLE); // Common elements
      sprite->drawString("Need Pass Key", 70, 20);
      break;
    case BMDCameraConnection::FailedPassKey:
      Screen_Common(TFT_ORANGE); // Common elements
      sprite->drawString("Wrong Pass Key", 70, 20);
      break;
    case BMDCameraConnection::Disconnected:
      DEBUG_DEBUG("NoConnection - Disconnected");
      Screen_Common(TFT_RED); // Common elements
      sprite->drawString("Disconnected (wait)", 70, 20);
      break;
    case BMDCameraConnection::IncompatibleProtocol:
      // Note: This needs to be worked on as there's no incompatible protocol connections yet.
      Screen_Common(TFT_RED); // Common elements
      sprite->drawString("Incompatible Protocol", 70, 20);
      break;
    default:
      break;
  }

  // Show up to two cameras
  int cameras = cameraConnection.cameraAddresses.size();
  for(int count = 0; count < cameras && count < 2; count++)
  {
    // Cameras
    sprite->fillRoundRect(25 + (count * 125) + (count * 10), 60, 125, 100, 5, TFT_DARKGREY);

    // Highlight the camera to connect to
    if(connectToCameraIndex != -1 && connectToCameraIndex == count)
    {
      sprite->drawRoundRect(25 + (count * 125) + (count * 10), 60, 5, 2, TFT_GREEN);
    }

    sprite->pushImage(33 + (count * 125) + (count * 10), 69, 110, 61, blackmagic_pocket_4k_110x61);

    sprite->drawString(cameraConnection.cameraAddresses[count].toString().c_str(), 33 + (count * 125) + (count * 10), 144, &Lato_Regular6pt7b);
  }

  // If there's more than one camera, check for a tap to see if they have nominated one to connect to
  /*
  if(cameras > 1 && tapped_x != -1)
  {
      if(tapped_x >= 25 && tapped_y >= 60 && tapped_x <= 150 && tapped_y <= 160)
      {
        // First camera tapped
        connectToCameraIndex = 0;
      }
      else if(tapped_x >= 160 && tapped_y >= 60 && tapped_x <= 285 && tapped_y <= 160)
      {
        // Second camera tapped
        connectToCameraIndex = 1;
      }
  }
  */

  if(connectToCameraIndex != -1)
  {
    cameraConnection.connect(cameraConnection.cameraAddresses[connectToCameraIndex]);
    connectToCameraIndex = -1;

    if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::FailedPassKey)
    {
      DEBUG_DEBUG("NoConnection - Failed Pass Key");
    }
  }

  sprite->pushSprite(0, 0);
}

short testFocusPosition = 18;

// Default screen for connected state
void Screen_Dashboard(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Dashboard;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  int xshift = 0;

  bool tappedAction = false;

  if(btnAPressed)
  {
    // TESTING TESTING TESTING
    for(float x = 0.0; x < 1.0; x += 0.01)
    {
      // PacketWriter::writeFocusPosition(x, &cameraConnection);
      PacketWriter::writeZoomNormalised(x, &cameraConnection);

      DEBUG_DEBUG(std::to_string(x).c_str());

      delay(1000);
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh)
  // if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  // DEBUG_DEBUG("Screen Dashboard Refreshing.");

  if(!sprite->createSprite(IWIDTH_SPRITE, IHEIGHT_SPRITE)) return;

  if(cameraConnection.getInitialPayloadTime() != ULONG_MAX)
    sprite->fillSprite(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // Set font here rather than on each drawString line
  sprite->setFont(&Lato_Regular11pt7b);

  // ISO
  if(camera->hasSensorGainISOValue())
  {
    sprite->fillSmoothRoundRect(20, 5, 75, 65, 3, TFT_DARKGREY);
    sprite->setTextColor(TFT_WHITE);

    sprite->drawCentreString(String(camera->getSensorGainISOValue()), 58, 23);

    sprite->drawCentreString("ISO", 58, 50, &AgencyFB_Regular7pt7b);
  }

  // Shutter
  xshift = 80;
  if(camera->hasShutterAngle() || camera->hasShutterSpeed())
  {
    sprite->fillSmoothRoundRect(20 + xshift, 5, 75, 65, 3, TFT_DARKGREY);
    sprite->setTextColor(TFT_WHITE);

    if(camera->shutterValueIsAngle && camera->hasShutterAngle())
    {
      // Shutter Angle
      int currentShutterAngle = camera->getShutterAngle();
      float ShutterAngleFloat = currentShutterAngle / 100.0;

      sprite->drawCentreString(String(ShutterAngleFloat, (currentShutterAngle % 100 == 0 ? 0 : 1)), 58 + xshift, 23);
    }
    else if(camera->hasShutterSpeed())
    {
      // Shutter Speed
      int currentShutterSpeed = camera->getShutterSpeed();

      sprite->drawCentreString("1/" + String(currentShutterSpeed), 58 + xshift, 23);
    }

    sprite->drawCentreString(camera->shutterValueIsAngle ? "DEGREES" : "SPEED", 58 + xshift, 50, &AgencyFB_Regular7pt7b); //  "SHUTTER"
  }

  // WhiteBalance and Tint
  xshift += 80;
  if(camera->hasWhiteBalance() || camera->hasTint())
  {
    sprite->fillSmoothRoundRect(20 + xshift, 5, 135, 65, 3, TFT_DARKGREY);
    sprite->setTextColor(TFT_WHITE);

    if(camera->hasWhiteBalance())
      sprite->drawCentreString(String(camera->getWhiteBalance()), 58 + xshift, 23);

    sprite->drawCentreString("WB", 58 + xshift, 50, &AgencyFB_Regular7pt7b);

    xshift += 66;

    if(camera->hasTint())
      sprite->drawCentreString(String(camera->getTint()), 58 + xshift, 23);

    sprite->drawCentreString("TINT", 58 + xshift, 50, &AgencyFB_Regular7pt7b);
  }

  // Codec
  if(camera->hasCodec())
  {
    sprite->fillSmoothRoundRect(20, 75, 155, 40, 3, TFT_DARKGREY);

    sprite->drawCentreString(camera->getCodec().to_string().c_str(), 97, 84);
  }

  // Media
  if(camera->getMediaSlots().size() != 0)
  {
    std::string slotString;

    for(int i = 0; i < camera->getMediaSlots().size(); i++)
    {
      if(camera->getMediaSlots()[i].active)
      {
        switch(camera->getMediaSlots()[i].medium)
        {
          case CCUPacketTypes::ActiveStorageMedium::CFastCard:
            slotString = "CFAST";
            break;
          case CCUPacketTypes::ActiveStorageMedium::SDCard:
            slotString = "SD";
            break;
          case CCUPacketTypes::ActiveStorageMedium::SSDRecorder:
            slotString = "SSD";
            break;
          case CCUPacketTypes::ActiveStorageMedium::USB:
            slotString = "USB";
            break;
        }

        if(!slotString.empty())
          break;
      }
    }

    if(!slotString.empty())
    {
      sprite->fillSmoothRoundRect(20, 120, 100, 40, 3, TFT_DARKGREY);

      sprite->drawCentreString(slotString.c_str(), 70, 130);

      // Show recording error
      if(camera->hasRecordError())
        sprite->drawRoundRect(20, 120, 100, 40, 3, TFT_RED);
    }
    else
    {
      // Show no Media
      sprite->fillSmoothRoundRect(20, 120, 100, 40, 3, TFT_DARKGREY);
      sprite->drawCentreString("NO MEDIA", 70, 135, &AgencyFB_Regular7pt7b);
    }
  }

  // Recording Format - Frame Rate and Resolution
  if(camera->hasRecordingFormat())
  {
    // Frame Rate
    sprite->fillSmoothRoundRect(180, 75, 135, 40, 3, TFT_DARKGREY);

    sprite->drawCentreString(camera->getRecordingFormat().frameRate_string().c_str(), 237, 84);

    sprite->drawCentreString("fps", 300, 89, &AgencyFB_Regular7pt7b);

    // Resolution
    sprite->fillSmoothRoundRect(125, 120, 190, 40, 3, TFT_DARKGREY);

    std::string resolution = camera->getRecordingFormat().frameDimensionsShort_string();
    sprite->drawCentreString(resolution.c_str(), 220, 130);
  }

  // Lens
  if(camera->hasFocalLengthMM() && camera->hasApertureFStopString())
  {
    // Lens
    sprite->fillSmoothRoundRect(20, 165, 295, 40, 3, TFT_DARKGREY);

    if(camera->hasFocalLengthMM() && camera->hasApertureFStopString())
    {
      auto focalLength = camera->getFocalLengthMM();
      std::string focalLengthMM = std::to_string(focalLength);
      std::string combined = focalLengthMM + "mm";

      sprite->drawString(combined.c_str(), 30, 174);
      sprite->drawString(camera->getApertureFStopString().c_str(), 100, 174);
    }
  }

  sprite->pushSprite(0, 0);
}

void Screen_Recording(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Recording;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // If we have a tap, we should determine if it is on anything
  /*
  bool tappedAction = false;
  if(tapped_x != -1 && camera->hasTransportMode())
  {
    if(tapped_x >= 195 && tapped_y <= 128)
    {
      // Record button
      auto transportInfo = camera->getTransportMode();

      if(camera->isRecording)
      {
        DEBUG_VERBOSE("Record Stop");
        transportInfo.mode = CCUPacketTypes::MediaTransportMode::Preview;
      }
      else
      {
        DEBUG_VERBOSE("Record Start");
        transportInfo.mode = CCUPacketTypes::MediaTransportMode::Record;
      }

      PacketWriter::writeTransportInfo(transportInfo, &cameraConnection);

      tappedAction = true;
    }
  }
  */

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh)
  // if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();

  DEBUG_DEBUG("Screen Recording Refreshed.");

  sprite->fillSprite(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // M5GFX, set font here rather than on each drawString line
  sprite->setFont(&Lato_Regular11pt7b);

  // Record button
  if(camera->isRecording) sprite->fillSmoothCircle(257, 63, 58, TFT_RED); // Recording solid
  sprite->drawCircle(257, 63, 57, (camera->isRecording ? TFT_RED : TFT_LIGHTGREY)); // Outer
  sprite->fillSmoothCircle(257, 63, 38, camera->isRecording ? TFT_RED : TFT_LIGHTGREY); // Inner

  // Timecode
  sprite->setTextColor(camera->isRecording ? TFT_RED : TFT_WHITE);
  sprite->drawString(camera->getTimecodeString().c_str(), 30, 57);

  // Remaining time and any errors
  if(camera->getMediaSlots().size() != 0 && camera->hasActiveMediaSlot())
  {
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString((camera->getActiveMediaSlot().GetMediumString() + " " + camera->getActiveMediaSlot().remainingRecordTimeString).c_str(), 30, 130);

    sprite->drawString("REMAINING TIME", 30, 153, &Lato_Regular5pt7b);

    // Show any media record errors
    if(camera->hasRecordError())
    {
      sprite->setTextColor(TFT_RED);
      sprite->drawString("RECORD ERROR", 30, 20);
    }
  }

  sprite->pushSprite(0, 0);
}

// The Gyro Log screen. This is the default page when connected.
//
// It has two modes:
//   * Calibration mode (default): shows the live gyro (rad/s) and accel (g)
//     values so the user can lay the unit flat. A/B step the GCSV orientation.
//   * Summary mode (after a clip has been recorded): shows the duration, the
//     running timecode, the .gcsv file name, the file size written, and the
//     remaining space on the SD card.
//
// While a clip is actively being recorded the screen is turned off (the IMU
// is being sampled and written to the SD card), so this screen is blank by
// design during that time.
void Screen_GyroLog(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::GyroLog;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Handle the A/B buttons: step the GCSV orientation (calibration).
  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("GyroLog: Btn A/B pressed (orientation)");

    int idx = gyroLog.getOrientationIndex();
    if(btnAPressed)
      idx = (idx - 1 + GyroLogWriter::kOrientationCount) % GyroLogWriter::kOrientationCount;
    else
      idx = (idx + 1) % GyroLogWriter::kOrientationCount;

    gyroLog.setOrientationIndex(idx);
    tappedAction = true;
  }

  // While actively recording the screen is turned off (the IMU is being
  // sampled and written to the SD card). Don't draw anything and don't touch
  // the display while in this state.
  if(gyroLog.isRecording())
    return;

  // The live IMU values change constantly, so always refresh unless the screen
  // content is unchanged and nothing was pressed.
  if(!forceRefresh && !tappedAction && lastRefreshedScreen == camera->getLastModified())
    return;
  lastRefreshedScreen = camera->getLastModified();

  sprite->fillSprite(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // Title
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("GYRO LOG", 30, 9, &AgencyFB_Bold9pt7b);

  // M5GFX, set font here rather than on each drawString line
  sprite->setFont(&Lato_Regular11pt7b);

  if(gyroLog.getSummary().valid)
  {
    // Summary mode: show the details of the last clip written.
    const GyroLogWriter::Summary& s = gyroLog.getSummary();

    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("LAST CLIP", 30, 40, &Lato_Regular5pt7b);

    // File name
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(s.fileName.c_str(), 30, 55, &Lato_Regular6pt7b);

    // Duration
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("DURATION", 30, 95, &Lato_Regular5pt7b);
    char durBuf[32];
    snprintf(durBuf, sizeof(durBuf), "%02lu:%02lu.%02lu",
      (unsigned long)(s.durationMs / 60000),
      (unsigned long)((s.durationMs / 1000) % 60),
      (unsigned long)((s.durationMs / 10) % 100));
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(durBuf, 30, 108, &Lato_Regular11pt7b);

    // Timecode at end
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("TIMECODE (END)", 30, 145, &Lato_Regular5pt7b);
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(s.timecodeAtEnd.c_str(), 30, 158, &Lato_Regular11pt7b);

    // File size
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("FILE SIZE", 30, 185, &Lato_Regular5pt7b);
    char sizeBuf[32];
    snprintf(sizeBuf, sizeof(sizeBuf), "%.2f MB", (double)s.fileSizeBytes / (1024.0 * 1024.0));
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(sizeBuf, 30, 198, &Lato_Regular11pt7b);

    // Free SD space. The value is computed in the background (see
    // gyroFreeSpacePending) because freeClusterCount() walks the whole FAT;
    // until that first computation finishes it reads as 0.
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("SD FREE", 180, 185, &Lato_Regular5pt7b);
    char freeBuf[32];
    snprintf(freeBuf, sizeof(freeBuf), "%.1f GB", (double)s.freeBytes / (1024.0 * 1024.0 * 1024.0));
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(freeBuf, 180, 198, &Lato_Regular11pt7b);
  }
  else
  {
    // Calibration mode: show the live IMU values and the current orientation.
    // We use M5Unified's calibrated, axis-ordered getGyro()/getAccel() here (not
    // gyroLog.readImu, which reads the raw registers directly) so the numbers
    // shown match what the user sees for orientation. The 256 us throttle in
    // those getters is fine for a calibration display.
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyro(&gx, &gy, &gz);
    M5.Imu.getAccel(&ax, &ay, &az);

    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("GYRO (rad/s)", 30, 40, &Lato_Regular5pt7b);
    char gBuf[64];
    snprintf(gBuf, sizeof(gBuf), "x %7.3f  y %7.3f  z %7.3f", gx, gy, gz);
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(gBuf, 30, 53, &Lato_Regular11pt7b);

    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("ACCEL (g)", 30, 95, &Lato_Regular5pt7b);
    char aBuf[64];
    snprintf(aBuf, sizeof(aBuf), "x %7.3f  y %7.3f  z %7.3f", ax, ay, az);
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString(aBuf, 30, 108, &Lato_Regular11pt7b);

    // Current orientation token (with its index, so it can be reported back)
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("ORIENTATION", 30, 150, &Lato_Regular5pt7b);
    sprite->setTextColor(TFT_CYAN);
    char orientBuf[24];
    snprintf(orientBuf, sizeof(orientBuf), "%s  (#%d)",
      GyroLogWriter::orientationToken(gyroLog.getOrientationIndex()),
      gyroLog.getOrientationIndex());
    sprite->drawString(orientBuf, 30, 163, &Lato_Regular11pt7b);

    // Hint
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("Lay flat, A/B to set orientation", 30, 195, &Lato_Regular5pt7b);

    // SD card status diagnostic (top-right). Green when ready, red otherwise.
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString("SD CARD", 230, 40, &Lato_Regular5pt7b);
    sprite->setTextColor(gyroLog.sdReady() ? TFT_GREEN : TFT_RED);
    sprite->drawString(gyroLog.sdStatusMessage().c_str(), 230, 53, &Lato_Regular6pt7b);
  }

  sprite->pushSprite(0, 0);
}

void Screen_ISO(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::ISO;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // If we have an up/down button press
  bool tappedAction = false;
  std::vector<int> Options_ISO = {200, 400, 640, 800, 1250, 3200, 8000, 12800};
  int Options_ISO_SelectingIndex = -1; // when using up/down buttons keeps track of what option you are navigating to
  if(btnAPressed || btnBPressed)
  {
    if(camera->hasSensorGainISOValue()) // Ensure we have the ISO value before allowing it to be changed
    {
      int currentValue = camera->getSensorGainISOValue();

      bool up = btnBPressed; // up = true, down = false

      // Find the closest ISO (particularly if it's not the standard set)
      int closestValue = Options_ISO[0];
      int minDifference = std::abs(currentValue - closestValue);

      for (int i = 1; i < Options_ISO.size(); ++i) {
          int difference = std::abs(currentValue - Options_ISO[i]);
          if (difference < minDifference) {
              minDifference = difference;
              closestValue = Options_ISO[i];
          }
      }

      auto iter = std::find(Options_ISO.begin(), Options_ISO.end(), closestValue);
      // Get the closest ISO
      if (iter != Options_ISO.end()) {
          Options_ISO_SelectingIndex = std::distance(Options_ISO.begin(), iter);
      }

      DEBUG_DEBUG("Options_ISO_SelectingIndex");
      DEBUG_DEBUG(std::to_string(Options_ISO_SelectingIndex).c_str());

      if(Options_ISO_SelectingIndex != -1)
      {
        Options_ISO_SelectingIndex = Options_ISO_SelectingIndex + (up ? 1 : -1); // Move the selected option

        if(Options_ISO_SelectingIndex < 0)
        {
          // Down button on the first option, go to the last option
          Options_ISO_SelectingIndex = Options_ISO.size() -1;
        }
        else if(Options_ISO_SelectingIndex > (Options_ISO.size() - 1))
        {
          // Up button on the last option, go to the first option
          Options_ISO_SelectingIndex = 0;
        }

        // ISO selected, send it to the camera
        PacketWriter::writeISO(Options_ISO[Options_ISO_SelectingIndex], &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen ISO Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // M5GFX, set font here rather than on each drawString line
  sprite->setFont(&Lato_Regular11pt7b);

  // Get the current ISO value
  int currentISO = 0;
  if(camera->hasSensorGainISOValue())
    currentISO = camera->getSensorGainISOValue();

  // ISO label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("ISO", 30, 9, &AgencyFB_Bold9pt7b);

  // sprite->textbgcolor = TFT_DARKGREY;

  // 200
  int labelISO = 200;
  sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 65, 41);

  // 400
  labelISO = 400;
  sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 160, 36);
  sprite->drawCentreString("NATIVE", 160, 59, &Lato_Regular5pt7b);

  // 8000
  labelISO = 8000;
  sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 260, 41);

  // 640
  labelISO = 640;
  sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 65, 87);

  // 800
  labelISO = 800;
  sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 160, 87);

  // 12800
  labelISO = 12800;
  sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 260, 87);

  // 1250
  labelISO = 1250;
  sprite->fillSmoothRoundRect(20, 120, 90, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 65, 131);

  // 3200
  labelISO = 3200;
  sprite->fillSmoothRoundRect(115, 120, 90, 40, 3, (currentISO == labelISO ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(String(labelISO).c_str(), 160, 126);
  sprite->drawCentreString("NATIVE", 160, 149, &Lato_Regular5pt7b);

  // Custom ISO - show if ISO is not one of the above values
  if(currentISO != 0)
  {
    // Only show the ISO value if it's not a standard one
    if(currentISO != 200 && currentISO != 400 && currentISO != 640 && currentISO != 800 && currentISO != 1250 && currentISO != 3200 && currentISO != 8000 && currentISO != 12800)
    {
      sprite->fillSmoothRoundRect(210, 120, 100, 40, 3, TFT_DARKGREEN);
      // sprite->textbgcolor = TFT_DARKGREEN;
      sprite->drawCentreString(String(currentISO).c_str(), 260, 126);
      sprite->drawCentreString("CUSTOM", 260, 149, &Lato_Regular5pt7b);
    }
  }

  sprite->pushSprite(0, 0);
}

void Screen_ShutterAngle(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::ShutterAngleSpeed;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // If we have an up/down button press
  bool tappedAction = false;
  std::vector<int> Options_ShutterAngle = {1500, 6000, 9000, 12000, 15000, 18000, 27000, 36000}; // Values are X100
  int Options_ShutterAngle_SelectingIndex = -1; // when using up/down buttons keeps track of what option you are navigating to
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Shutter Angle Screen: Btn A/B pressed");

    if(camera->hasShutterAngle()) // Ensure we have a value before allowing it to be changed
    {
      // Get the current Shutter Angle (comes through X100, so 180 degrees = 18000)
      int currentValue = camera->getShutterAngle();

      bool up = btnBPressed; // up = true, down = false

      // If we don't have a selected value yet, set to the current one
      if(Options_ShutterAngle_SelectingIndex == -1)
      {
        auto iter = std::find(Options_ShutterAngle.begin(), Options_ShutterAngle.end(), currentValue);
        int index = (iter != Options_ShutterAngle.end()) ? std::distance(Options_ShutterAngle.begin(), iter) : -1;

        if(index != -1)
          Options_ShutterAngle_SelectingIndex = index;
        else // Custom Shutter Angle, use the latest one
        {
          int closestIndex = -1;
          int closestValue = std::numeric_limits<int>::min();  // Initialize with the lowest possible value

          for (int i = 0; i < Options_ShutterAngle.size(); ++i) {
              int optionValue = Options_ShutterAngle[i];

              if (optionValue <= currentValue && optionValue > closestValue) {
                  closestIndex = i;
                  closestValue = optionValue;
              }
          }

          if (closestIndex != -1)
          {
            if(up)
              Options_ShutterAngle_SelectingIndex = closestIndex; // Pressing up, closest one lower is right
            else
            {
              if(closestIndex < Options_ShutterAngle.size() - 1)
                Options_ShutterAngle_SelectingIndex = closestIndex + 1; // Pressing down, go up
              else
                Options_ShutterAngle_SelectingIndex = 0; // Already at the top, so go back to the start
            }
          }
          else // Lower than the lowest, so set to lowest in options
          {
            if(up)
              Options_ShutterAngle_SelectingIndex = Options_ShutterAngle.size() - 1; // Pressing up on a number lower than the lowest, put on the highest so we can go back around to the lowest in the set
            else
              Options_ShutterAngle_SelectingIndex = 0; // Otherwise pressing down and keep at the lowest and we'll be going to the highest
          }
        }
      }

      // Move to the next option and send it to the camera
      if(Options_ShutterAngle_SelectingIndex != -1)
      {
        Options_ShutterAngle_SelectingIndex = Options_ShutterAngle_SelectingIndex + (up ? 1 : -1); // Move the selected option

        if(Options_ShutterAngle_SelectingIndex < 0)
        {
          // Down button on the first option, go to the last option
          Options_ShutterAngle_SelectingIndex = Options_ShutterAngle.size() -1;
        }
        else if(Options_ShutterAngle_SelectingIndex > (Options_ShutterAngle.size() - 1))
        {
          // Up button on the last option, go to the first option
          Options_ShutterAngle_SelectingIndex = 0;
        }

        // Shutter Angle selected, send it to the camera
        PacketWriter::writeShutterAngle(Options_ShutterAngle[Options_ShutterAngle_SelectingIndex], &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Shutter Angle Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // Get the current Shutter Angle (comes through X100, so 180 degrees = 18000)
  int currentShutterAngle = 0;
  if(camera->hasShutterAngle())
    currentShutterAngle = camera->getShutterAngle();

  // Shutter Angle label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("DEGREES", 265, 9, &AgencyFB_Regular7pt7b);
  sprite->drawString("SHUTTER ANGLE", 30, 9, &AgencyFB_Bold9pt7b);

  // 15
  int labelShutterAngle = 1500;
  sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 0) sprite->drawRoundRect(20, 30, 90, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 65, 41);

  // 60
  labelShutterAngle = 6000;
  sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 1) sprite->drawRoundRect(115, 30, 90, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 160, 41);

  // 90
  labelShutterAngle = 9000;
  sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 2) sprite->drawRoundRect(210, 30, 100, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 260, 41);

  // 120
  labelShutterAngle = 12000;
  sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 3) sprite->drawRoundRect(20, 75, 90, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 65, 87);

  // 150
  labelShutterAngle = 15000;
  sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 4) sprite->drawRoundRect(115, 75, 90, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 160, 87);

  // 180 (with a border around it)
  labelShutterAngle = 18000;
  sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawRoundRect(210, 75, 100, 40, 3, TFT_DARKGREEN);
  if(Options_ShutterAngle_SelectingIndex == 5) sprite->drawRoundRect(210, 75, 100, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 260, 87);

  // 270
  labelShutterAngle = 27000;
  sprite->fillSmoothRoundRect(20, 120, 90, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 6) sprite->drawRoundRect(20, 120, 90, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 65, 131);

  // 360
  labelShutterAngle = 36000;
  sprite->fillSmoothRoundRect(115, 120, 90, 40, 3, (currentShutterAngle == labelShutterAngle ? TFT_DARKGREEN : TFT_DARKGREY));
  if(Options_ShutterAngle_SelectingIndex == 7) sprite->drawRoundRect(115, 120, 90, 40, 1, TFT_WHITE);
  sprite->drawCentreString(String(labelShutterAngle / 100).c_str(), 160, 131);

  // Custom Shutter Angle - show if not one of the above values
  if(currentShutterAngle != 0)
  {
    // Only show the Shutter angle value if it's not a standard one
    if(currentShutterAngle != 1500 && currentShutterAngle != 6000 && currentShutterAngle != 9000 && currentShutterAngle != 12000 && currentShutterAngle != 15000 && currentShutterAngle != 18000 && currentShutterAngle != 27000 && currentShutterAngle != 36000)
    {
      float customShutterAngle = currentShutterAngle / 100.0;

      sprite->fillSmoothRoundRect(210, 120, 100, 40, 3, TFT_DARKGREEN);
      sprite->drawCentreString(String(customShutterAngle, (currentShutterAngle % 100 == 0 ? 0 : 1)).c_str(), 260, 126);
      sprite->drawCentreString("CUSTOM", 260, 149, &Lato_Regular5pt7b);
    }
  }

  sprite->pushSprite(0, 0);
}

void Screen_ShutterSpeed(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::ShutterAngleSpeed;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Shutter Speed Values: 1/30, 1/50, 1/60, 1/125, 1/200, 1/250, 1/500, 1/2000, CUSTOM
  // Note that the protocol takes the denominator as its parameter value. So for 1/60 we'll pass 60.

  // If we have an up/down button press
  bool tappedAction = false;
  std::vector<int> Options_ShutterSpeed = {30, 50, 60, 125, 200, 250, 500, 2000};
  int Options_ShutterSpeed_SelectingIndex = -1; // when using up/down buttons keeps track of what option you are navigating to
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Shutter Speed Screen: Btn A/B pressed");

    if(camera->hasShutterSpeed()) // Ensure we have a value before allowing it to be changed
    {
      // Get the current Shutter Speed (denominator, e.g. 1/60 will return 60)
      int currentValue = camera->getShutterSpeed();

      bool up = btnBPressed; // up = true, down = false

      // If we don't have a selected value yet, set to the current one
      if(Options_ShutterSpeed_SelectingIndex == -1)
      {
        auto iter = std::find(Options_ShutterSpeed.begin(), Options_ShutterSpeed.end(), currentValue);
        int index = (iter != Options_ShutterSpeed.end()) ? std::distance(Options_ShutterSpeed.begin(), iter) : -1;

        if(index != -1)
          Options_ShutterSpeed_SelectingIndex = index;
        else // Custom Shutter Speed, use the latest one
        {
          int closestIndex = -1;
          int closestValue = std::numeric_limits<int>::min();  // Initialize with the lowest possible value

          for (int i = 0; i < Options_ShutterSpeed.size(); ++i) {
              int optionValue = Options_ShutterSpeed[i];

              if (optionValue <= currentValue && optionValue > closestValue) {
                  closestIndex = i;
                  closestValue = optionValue;
              }
          }

          if (closestIndex != -1)
          {
            if(up)
              Options_ShutterSpeed_SelectingIndex = closestIndex; // Pressing up, closest one lower is right
            else
            {
              if(closestIndex < Options_ShutterSpeed.size() - 1)
                Options_ShutterSpeed_SelectingIndex = closestIndex + 1; // Pressing down, go up
              else
                Options_ShutterSpeed_SelectingIndex = 0; // Already at the top, so go back to the start
            }
          }
          else // Lower than the lowest, so set to lowest in options
          {
            if(up)
              Options_ShutterSpeed_SelectingIndex = Options_ShutterSpeed.size() - 1; // Pressing up on a number lower than the lowest, put on the highest so we can go back around to the lowest in the set
            else
              Options_ShutterSpeed_SelectingIndex = 0; // Otherwise pressing down and keep at the lowest and we'll be going to the highest
          }
        }
      }

      // Move to the next option and send it to the camera
      if(Options_ShutterSpeed_SelectingIndex != -1)
      {
        Options_ShutterSpeed_SelectingIndex = Options_ShutterSpeed_SelectingIndex + (up ? 1 : -1); // Move the selected option

        if(Options_ShutterSpeed_SelectingIndex < 0)
        {
          // Down button on the first option, go to the last option
          Options_ShutterSpeed_SelectingIndex = Options_ShutterSpeed.size() -1;
        }
        else if(Options_ShutterSpeed_SelectingIndex > (Options_ShutterSpeed.size() - 1))
        {
          // Up button on the last option, go to the first option
          Options_ShutterSpeed_SelectingIndex = 0;
        }

        // Shutter Speed selected, send it to the camera
        PacketWriter::writeShutterSpeed(Options_ShutterSpeed[Options_ShutterSpeed_SelectingIndex], &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Shutter Speed Refreshed.");


  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // Get the current Shutter Speed
  int currentShutterSpeed = 0;
  if(camera->hasShutterSpeed())
    currentShutterSpeed = camera->getShutterSpeed();
  else
    DEBUG_DEBUG("DO NOT HAVE SHUTTER SPEED!");

  // Shutter Speed label
  sprite->setTextColor(TFT_WHITE);

  if(camera->hasRecordingFormat())
  {
    sprite->drawRightString(camera->getRecordingFormat().frameRate_string().c_str() + String(" fps"), 310, 9, &Lato_Regular5pt7b);
  }

  sprite->drawString("SHUTTER SPEED", 30, 9, &AgencyFB_Bold9pt7b);

  // 1/30
  int labelShutterSpeed = 30;
  sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 65, 41);

  // 1/50
  labelShutterSpeed = 50;
  sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 160, 41);

  // 1/60
  labelShutterSpeed = 60;
  sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 260, 41);

  // 1/125
  labelShutterSpeed = 125;
  sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 65, 87);

  // 1/200
  labelShutterSpeed = 200;
  sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 160, 87);

  // 1/250
  labelShutterSpeed = 250;
  sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 260, 87);

  // 1/500
  labelShutterSpeed = 500;
  sprite->fillSmoothRoundRect(20, 120, 90, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 65, 131);

  // 1/2000
  labelShutterSpeed = 2000;
  sprite->fillSmoothRoundRect(115, 120, 90, 40, 3, (currentShutterSpeed == labelShutterSpeed ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("1/" + String(labelShutterSpeed), 160, 131);

  // Custom Shutter Speed - show if not one of the above values
  if(currentShutterSpeed != 0)
  {
    // Only show the Shutter Speed value if it's not a standard one
    if(currentShutterSpeed != 30 && currentShutterSpeed != 50 && currentShutterSpeed != 60 && currentShutterSpeed != 125 && currentShutterSpeed != 200 && currentShutterSpeed != 250 && currentShutterSpeed != 500 && currentShutterSpeed != 2000)
    {
      sprite->fillSmoothRoundRect(210, 120, 100, 40, 3, TFT_DARKGREEN);
      sprite->drawCentreString("1/" + String(currentShutterSpeed), 260, 126);
      sprite->drawCentreString("CUSTOM", 260, 149, &Lato_Regular5pt7b);
    }
  }

  sprite->pushSprite(0, 0);
}

void Screen_WBTint(bool editWB, bool forceRefresh = false) // editWB indicates editing White Balance when true, editing Tint when false
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = editWB ? Screens::WhiteBalanceTintWB : Screens::WhiteBalanceTintT;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // White Balance and Tint
  // WB: Bright, Incandescent, Fluorescent, Mixed Light, Cloud

  // Get the current White Balance value
  int currentWB = 0;
  if(camera->hasWhiteBalance())
    currentWB = camera->getWhiteBalance();

  // Get the current Tint value
  int currentTint = 0;
  if(camera->hasTint())
    currentTint = camera->getTint();

  // If we have an up/down button press
  bool tappedAction = false;
  if((btnAPressed || btnBPressed) && camera->hasWhiteBalance() && camera->hasTint())
  {
    DEBUG_DEBUG("White Balance Screen: Btn A/B pressed");

    if(editWB) // White Balance Edit
    {
      if(btnAPressed)
      {
        // White Balance Presets

        std::vector<int> Options_WB = {5600, 3200, 4000, 4500, 6500};
        std::vector<int> Options_WB_Tint = {10, 0, 15, 15, 10}; // Corresponding Tints for WB presets
        int Options_WB_SelectingIndex = -1; // when using up/down buttons keeps track of what option you are navigating to

        // Get the current White Balance
        int currentValue = currentWB;

        bool up = true;

        // If we don't have a selected value yet, set to the current one
        if(Options_WB_SelectingIndex == -1)
        {
          auto iter = std::find(Options_WB.begin(), Options_WB.end(), currentValue);
          int index = (iter != Options_WB.end()) ? std::distance(Options_WB.begin(), iter) : -1;

          if(index != -1)
            Options_WB_SelectingIndex = index;
          else // Custom Shutter Speed, use the latest one
          {
            int closestIndex = -1;
            int closestValue = std::numeric_limits<int>::min();  // Initialize with the lowest possible value

            for (int i = 0; i < Options_WB.size(); ++i) {
                int optionValue = Options_WB[i];

                if (optionValue <= currentValue && optionValue > closestValue) {
                    closestIndex = i;
                    closestValue = optionValue;
                }
            }

            if (closestIndex != -1)
            {
              if(up)
                Options_WB_SelectingIndex = closestIndex; // Pressing up, closest one lower is right
              else
              {
                if(closestIndex < Options_WB.size() - 1)
                  Options_WB_SelectingIndex = closestIndex + 1; // Pressing down, go up
                else
                  Options_WB_SelectingIndex = 0; // Already at the top, so go back to the start
              }
            }
            else // Lower than the lowest, so set to lowest in options
            {
              if(up)
                Options_WB_SelectingIndex = Options_WB.size() - 1; // Pressing up on a number lower than the lowest, put on the highest so we can go back around to the lowest in the set
              else
                Options_WB_SelectingIndex = 0; // Otherwise pressing down and keep at the lowest and we'll be going to the highest
            }
          }
        }

        // Move to the next option and send it to the camera
        if(Options_WB_SelectingIndex != -1)
        {
          Options_WB_SelectingIndex = Options_WB_SelectingIndex + (up ? 1 : -1); // Move the selected option

          if(Options_WB_SelectingIndex < 0)
          {
            // Down button on the first option, go to the last option
            Options_WB_SelectingIndex = Options_WB.size() -1;
          }
          else if(Options_WB_SelectingIndex > (Options_WB.size() - 1))
          {
            // Up button on the last option, go to the first option
            Options_WB_SelectingIndex = 0;
          }

          // Shutter Speed selected, send it to the camera
          PacketWriter::writeWhiteBalance(Options_WB[Options_WB_SelectingIndex], Options_WB_Tint[Options_WB_SelectingIndex], &cameraConnection);

          tappedAction = true;
        }
      }
      else
      {
        // White Balance +100
        int newWB = currentWB + 100;

        if(newWB > 10000)
          newWB = 2500;

          // Shutter Speed selected, send it to the camera
          PacketWriter::writeWhiteBalance(newWB, currentTint, &cameraConnection);

          tappedAction = true;
      }
    }
    else if(!editWB && camera->hasTint())
    {
      std::vector<int> Options_Tint = {-50, -40, -30, -20, -15, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 30, 40, 50};
      int Options_Tint_SelectingIndex = -1; // when using up/down buttons keeps track of what option you are navigating to

      // Get the current Tint
      int currentValue = currentTint;

      bool up = btnBPressed; // up = true, down = false

      // If we don't have a selected value yet, set to the current one
      if(Options_Tint_SelectingIndex == -1)
      {
        auto iter = std::find(Options_Tint.begin(), Options_Tint.end(), currentValue);
        int index = (iter != Options_Tint.end()) ? std::distance(Options_Tint.begin(), iter) : -1;

        if(index != -1)
          Options_Tint_SelectingIndex = index;
        else // Custom Shutter Speed, use the latest one
        {
          int closestIndex = -1;
          int closestValue = std::numeric_limits<int>::min();  // Initialize with the lowest possible value

          for (int i = 0; i < Options_Tint.size(); ++i) {
              int optionValue = Options_Tint[i];

              if (optionValue <= currentValue && optionValue > closestValue) {
                  closestIndex = i;
                  closestValue = optionValue;
              }
          }

          if (closestIndex != -1)
          {
            if(up)
              Options_Tint_SelectingIndex = closestIndex; // Pressing up, closest one lower is right
            else
            {
              if(closestIndex < Options_Tint.size() - 1)
                Options_Tint_SelectingIndex = closestIndex + 1; // Pressing down, go up
              else
                Options_Tint_SelectingIndex = 0; // Already at the top, so go back to the start
            }
          }
          else // Lower than the lowest, so set to lowest in options
          {
            if(up)
              Options_Tint_SelectingIndex = Options_Tint.size() - 1; // Pressing up on a number lower than the lowest, put on the highest so we can go back around to the lowest in the set
            else
              Options_Tint_SelectingIndex = 0; // Otherwise pressing down and keep at the lowest and we'll be going to the highest
          }
        }
      }

      // Move to the next option and send it to the camera
      if(Options_Tint_SelectingIndex != -1)
      {
        Options_Tint_SelectingIndex = Options_Tint_SelectingIndex + (up ? 1 : -1); // Move the selected option

        if(Options_Tint_SelectingIndex < 0)
        {
          // Down button on the first option, go to the last option
          Options_Tint_SelectingIndex = Options_Tint.size() -1;
        }
        else if(Options_Tint_SelectingIndex > (Options_Tint.size() - 1))
        {
          // Up button on the last option, go to the first option
          Options_Tint_SelectingIndex = 0;
        }

        // Shutter Speed selected, send it to the camera
        PacketWriter::writeWhiteBalance(currentWB, Options_Tint[Options_Tint_SelectingIndex], &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen WB Tint Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // ISO label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString(editWB ? "WHITE BALANCE" : "TINT", 30, 9, &AgencyFB_Bold9pt7b);
  sprite->drawCentreString("TINT", 54, 132, &AgencyFB_Bold9pt7b);

  // Bright, 5600K
  int lblWBKelvin = 5600;
  int lblTint = 10;
  sprite->fillSmoothRoundRect(20, 30, 70, 40, 3, (currentWB == lblWBKelvin && currentTint == lblTint ? TFT_DARKGREEN : TFT_DARKGREY));
  if(currentWB == lblWBKelvin && currentTint == lblTint)
    sprite->pushImage(40, 35, 30, 30, WBBrightBG);
  else
    sprite->pushImage(40, 35, 30, 30, WBBright);

  // Incandescent, 3200K
  lblWBKelvin = 3200;
  lblTint = 0;
  sprite->fillSmoothRoundRect(95, 30, 70, 40, 3, (currentWB == lblWBKelvin && currentTint == lblTint ? TFT_DARKGREEN : TFT_DARKGREY));
  if(currentWB == lblWBKelvin && currentTint == lblTint)
    sprite->pushImage(115, 35, 30, 30, WBIncandescentBG);
  else
    sprite->pushImage(115, 35, 30, 30, WBIncandescent);

  // Fluorescent, 4000K
  lblWBKelvin = 4000;
  lblTint = 15;
  sprite->fillSmoothRoundRect(170, 30, 70, 40, 3, (currentWB == lblWBKelvin && currentTint == lblTint ? TFT_DARKGREEN : TFT_DARKGREY));
  if(currentWB == lblWBKelvin && currentTint == lblTint)
    sprite->pushImage(190, 35, 30, 30, WBFlourescentBG);
  else
    sprite->pushImage(190, 35, 30, 30, WBFlourescent);

  // Mixed Light, 4500K
  lblWBKelvin = 4500;
  lblTint = 15;
  sprite->fillSmoothRoundRect(245, 30, 70, 40, 3, (currentWB == lblWBKelvin && currentTint == lblTint ? TFT_DARKGREEN : TFT_DARKGREY));
  if(currentWB == lblWBKelvin && currentTint == lblTint)
    sprite->pushImage(265, 35, 30, 30, WBMixedLightBG);
  else
    sprite->pushImage(265, 35, 30, 30, WBMixedLight);

  // Cloud, 6500K
  lblWBKelvin = 6500;
  lblTint = 10;
  sprite->fillSmoothRoundRect(20, 75, 70, 40, 3, (currentWB == lblWBKelvin && currentTint == lblTint ? TFT_DARKGREEN : TFT_DARKGREY));
  if(currentWB == lblWBKelvin && currentTint == lblTint)
    sprite->pushImage(40, 80, 30, 30, WBCloudBG);
  else
    sprite->pushImage(40, 80, 30, 30, WBCloud);

  // Current White Balance Kelvin
  sprite->fillSmoothRoundRect(160, 75, 90, 40, 3, editWB ? TFT_DARKGREEN : TFT_DARKGREY);
  sprite->drawCentreString(String(currentWB), 205, 80);
  sprite->drawCentreString("KELVIN", 205, 103, &Lato_Regular5pt7b);

  if(editWB)
  {
    // WB Adjust Left <
    sprite->fillSmoothRoundRect(95, 75, 60, 40, 3, TFT_DARKGREY);
    sprite->drawCentreString("<", 125, 87);

    // WB Adjust Right >
    sprite->fillSmoothRoundRect(255, 75, 60, 40, 3, TFT_DARKGREY);
    sprite->drawCentreString(">", 284, 87);
  }

  // Current Tint
  sprite->fillSmoothRoundRect(160, 120, 90, 40, 3, !editWB ? TFT_DARKGREEN : TFT_DARKGREY);
  sprite->drawCentreString(String(currentTint), 205, 130);

  if(!editWB)
  {
    // Tint Adjust Left <
    sprite->fillSmoothRoundRect(95, 120, 60, 40, 3, TFT_DARKGREY);
    sprite->drawCentreString("<", 125, 132);

    // Tint Adjust Right >
    sprite->fillSmoothRoundRect(255, 120, 60, 40, 3, TFT_DARKGREY);
    sprite->drawCentreString(">", 284, 132);
  }

  sprite->pushSprite(0, 0);
}

// Codec Screen for Pocket 4K and 6K + Variants
void Screen_Codec4K6K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Codec;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present (the screen is blanked
  // below in that case anyway).
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  // Codec: BRAW and ProRes

  // If we have an up/down button press
  bool tappedAction = false;
  if((btnAPressed || btnBPressed) && camera->hasCodec())
  {
    DEBUG_DEBUG("Codec 4K/6K: Btn A/B pressed");

    // Switching between BRAW and ProRes
    if(btnAPressed)
    {
      if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
      {
        // Switch to ProRes
        PacketWriter::writeCodec(camera->lastKnownProRes, &cameraConnection);
      }
      else
      {
        // Switch to BRAW
        PacketWriter::writeCodec(camera->lastKnownBRAWIsBitrate ? camera->lastKnownBRAWBitrate : camera->lastKnownBRAWQuality, &cameraConnection);
      }

      tappedAction = true;
    }
    else if(btnBPressed)
    {
        // Current setting
        std::string currentCodecString = currentCodec.to_string();

      // Change the setting on the current Codec
      if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
      {
          if(currentCodecString == "BRAW 3:1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW5_1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW 5:1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW8_1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW 8:1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW12_1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW 12:1")
          {
            // Switch across to Constant Quality
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAWQ0), &cameraConnection);
          }
          else if(currentCodecString == "BRAW Q0")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAWQ1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW Q1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAWQ3), &cameraConnection);
          }
          else if(currentCodecString == "BRAW Q3")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAWQ5), &cameraConnection);
          }
          else if(currentCodecString == "BRAW Q5")
          {
            // Switch across to Constant Bitrate
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW3_1), &cameraConnection);
          }
      }
      else
      {
          if(currentCodecString == "ProRes HQ")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProRes422), &cameraConnection);
          }
          else if(currentCodecString == "ProRes 422")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProResLT), &cameraConnection);
          }
          else if(currentCodecString == "ProRes LT")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProResProxy), &cameraConnection);
          }
          else if(currentCodecString == "ProRes PXY")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProResHQ), &cameraConnection);
          }
      }

      tappedAction = true;
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Codec 4K/6K Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // We need to have the Codec information to show the screen
  if(!camera->hasCodec())
  {
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString("NO CODEC INFO.", 30, 9);

    return;
  }

  // Codec label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("CODEC", 30, 9, &AgencyFB_Bold9pt7b);

  // BRAW and ProRes selector buttons

  // BRAW
  sprite->fillSmoothRoundRect(20, 30, 145, 40, 5, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawRoundRect(20, 30, 145, 40, 3, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("BRAW", 93, 41);

  // ProRes
  sprite->fillSmoothRoundRect(170, 30, 145, 40, 5, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawRoundRect(170, 30, 145, 40, 3, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("ProRes", 242, 41);

  if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
  {
    // BRAW

    // Are we Constant Bitrate or Constant Quality
    std::string currentCodecString = currentCodec.to_string();
    auto pos = std::find(currentCodecString.begin(), currentCodecString.end(), ':');
    bool isConstantBitrate = pos != currentCodecString.end(); // Is there a colon, :, in the string? If so, it's Constant Bitrate

    // Get the bitrate or quality setting
    std::size_t spaceIndex = currentCodecString.find(" ");
    std::string qualityBitrateSetting = currentCodecString.substr(spaceIndex + 1); // e.g. 3:1, Q3, etc.

    // Constant Bitrate
    sprite->fillSmoothRoundRect(20, 75, 145, 40, 3, (isConstantBitrate ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("BITRATE", 93, 80);
    sprite->drawCentreString("CONSTANT", 93, 102, &Lato_Regular5pt7b);

    // Constant Quality
    sprite->fillSmoothRoundRect(170, 75, 145, 40, 3, (!isConstantBitrate ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("QUALITY", 242, 80);
    sprite->drawCentreString("CONSTANT", 242, 102, &Lato_Regular5pt7b);

    // Setting 1 of 4
    std::string optionString = (isConstantBitrate ? "3:1" : "Q0");
    sprite->fillSmoothRoundRect(20, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 55, 131);

    // Setting 2 of 4
    optionString = (isConstantBitrate ? "5:1" : "Q1");
    sprite->fillSmoothRoundRect(95, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 130, 131);

  //   Setting 3 of 4
    optionString = (isConstantBitrate ? "8:1" : "Q3");
    sprite->fillSmoothRoundRect(170, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 205, 131);

    // Setting 4 of 4
    optionString = (isConstantBitrate ? "12:1" : "Q5");
    sprite->fillSmoothRoundRect(245, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 280, 131);
  }
  else
  {
    // ProRes

    // Get the ProRes Setting
    std::string currentCodecString = currentCodec.to_string();
    std::size_t spaceIndex = currentCodecString.find(" ");
    std::string currentProResSetting = currentCodecString.substr(spaceIndex + 1); // e.g. HQ, 422, LT, PXY

    // HQ
    std::string proResLabel = "HQ";
    sprite->fillSmoothRoundRect(20, 75, 145, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 93, 87);

    // 422
    proResLabel = "422";
    sprite->fillSmoothRoundRect(170, 75, 145, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 242, 87);

    // LT
    proResLabel = "LT";
    sprite->fillSmoothRoundRect(20, 120, 145, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 93, 131);

    // PXY
    proResLabel = "PXY";
    sprite->fillSmoothRoundRect(170, 120, 145, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 242, 131);
  }

  sprite->pushSprite(0, 0);
}

// Codec Screen for URSA Mini Pro G2
void Screen_CodecURSAMiniProG2(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Codec;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present.
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  // Codec: BRAW and ProRes

  // If we have an up/down button press
  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Codec URSA Mini Pro G2: Btn A/B pressed");

    // Switching between BRAW and ProRes
    if(btnAPressed)
    {
      if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
      {
        // Switch to ProRes
        PacketWriter::writeCodec(camera->lastKnownProRes, &cameraConnection);
      }
      else
      {
        // Switch to BRAW
        PacketWriter::writeCodec(camera->lastKnownBRAWIsBitrate ? camera->lastKnownBRAWBitrate : camera->lastKnownBRAWQuality, &cameraConnection);
      }

      tappedAction = true;
    }
    else if(btnBPressed)
    {
        // Current setting
        std::string currentCodecString = currentCodec.to_string();

      // Change the setting on the current Codec
      if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
      {
          if(currentCodecString == "BRAW 3:1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW5_1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW 5:1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW8_1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW 8:1")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW12_1), &cameraConnection);
          }
          else if(currentCodecString == "BRAW 12:1")
          {
            // Switch across to Constant Quality
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAWQ0), &cameraConnection);
          }
          else if(currentCodecString == "BRAW Q0")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAWQ5), &cameraConnection);
          }
          else if(currentCodecString == "BRAW Q5")
          {
            // Switch across to Constant Bitrate
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kBRAW3_1), &cameraConnection);
          }
      }
      else
      {
          if(currentCodecString == "ProRes 444XQ")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProRes444), &cameraConnection);
          }
          else if(currentCodecString == "ProRes 444")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProResHQ), &cameraConnection);
          }
          else if(currentCodecString == "ProRes HQ")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProRes422), &cameraConnection);
          }
          else if(currentCodecString == "ProRes 422")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProResLT), &cameraConnection);
          }
          else if(currentCodecString == "ProRes LT")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProResProxy), &cameraConnection);
          }
          else if(currentCodecString == "ProRes PXY")
          {
            PacketWriter::writeCodec(CodecInfo(CCUPacketTypes::BasicCodec::ProRes, CCUPacketTypes::CodecVariants::kProRes444XQ), &cameraConnection);
          }
      }

      tappedAction = true;
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Codec URSA Mini Pro G2 Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // We need to have the Codec information to show the screen
  if(!camera->hasCodec())
  {
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString("NO CODEC INFO.", 30, 9);

    return;
  }

  // Codec label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("CODEC", 30, 9, &AgencyFB_Bold9pt7b);

  // BRAW and ProRes selector buttons

  // BRAW
  sprite->fillSmoothRoundRect(20, 30, 145, 40, 5, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawRoundRect(20, 30, 145, 40, 3, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("BRAW", 93, 41);

  // ProRes
  sprite->fillSmoothRoundRect(170, 30, 145, 40, 5, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawRoundRect(170, 30, 145, 40, 3, (currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString("ProRes", 242, 41);

  if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
  {
    // BRAW

    // Are we Constant Bitrate or Constant Quality
    std::string currentCodecString = currentCodec.to_string();
    auto pos = std::find(currentCodecString.begin(), currentCodecString.end(), ':');
    bool isConstantBitrate = pos != currentCodecString.end(); // Is there a colon, :, in the string? If so, it's Constant Bitrate

    // Get the bitrate or quality setting
    std::size_t spaceIndex = currentCodecString.find(" ");
    std::string qualityBitrateSetting = currentCodecString.substr(spaceIndex + 1); // e.g. 3:1, Q3, etc.

    // Constant Bitrate
    sprite->fillSmoothRoundRect(20, 75, 145, 40, 3, (isConstantBitrate ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("BITRATE", 93, 80);
    sprite->drawCentreString("CONSTANT", 93, 102, &Lato_Regular5pt7b);

    // Constant Quality
    sprite->fillSmoothRoundRect(170, 75, 145, 40, 3, (!isConstantBitrate ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("QUALITY", 242, 80);
    sprite->drawCentreString("CONSTANT", 242, 102, &Lato_Regular5pt7b);

    // Setting 1 of 4
    std::string optionString = (isConstantBitrate ? "3:1" : "Q0");
    sprite->fillSmoothRoundRect(20, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 55, 131);

    // Setting 2 of 4
    optionString = (isConstantBitrate ? "5:1" : "Q1");
    sprite->fillSmoothRoundRect(95, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 130, 131);

  //   Setting 3 of 4
    optionString = (isConstantBitrate ? "8:1" : "Q3");
    sprite->fillSmoothRoundRect(170, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 205, 131);

    // Setting 4 of 4
    optionString = (isConstantBitrate ? "12:1" : "Q5");
    sprite->fillSmoothRoundRect(245, 120, 70, 40, 3, (optionString == qualityBitrateSetting ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(optionString.c_str(), 280, 131);
  }
  else
  {
    // ProRes

    // Get the ProRes Setting
    std::string currentCodecString = currentCodec.to_string();
    std::size_t spaceIndex = currentCodecString.find(" ");
    std::string currentProResSetting = currentCodecString.substr(spaceIndex + 1); // e.g. 444XQ, 444, HQ, 422, LT, PXY

    // 444 XQ
    std::string proResLabel = "444XQ";
    sprite->fillSmoothRoundRect(20, 75, 95, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("XQ", 67, 87);

    // 444
    proResLabel = "444";
    sprite->fillSmoothRoundRect(120, 75, 95, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 167, 87);

    // HQ
    proResLabel = "HQ";
    sprite->fillSmoothRoundRect(220, 75, 95, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 267, 87);

    // 422
    proResLabel = "422";
    sprite->fillSmoothRoundRect(20, 120, 95, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 67, 131);

    // LT
    proResLabel = "LT";
    sprite->fillSmoothRoundRect(120, 120, 95, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 167, 131);

    // PXY
    proResLabel = "PXY";
    sprite->fillSmoothRoundRect(220, 120, 95, 40, 3, (currentProResSetting == proResLabel ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(proResLabel.c_str(), 267, 131);
  }

  sprite->pushSprite(0, 0);
}

// Codec Screen for URSA Mini Pro 12K
void Screen_CodecURSAMiniPro12K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Codec;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  
  // TO DO
}

// Codec screen - redirects to appropriate screen for camera
void Screen_Codec(bool forceRefresh = false)
{
  auto camera = BMDControlSystem::getInstance()->getCamera();

  if(camera->hasCodec())
  {
    if(camera->hasModelName())
    {
      if(camera->isPocket4K6K())
        Screen_Codec4K6K(forceRefresh); // Pocket camera
      else if(camera->isURSAMiniProG2())
        Screen_CodecURSAMiniProG2(forceRefresh); // URSA Mini Pro G2
      else if(camera->isURSAMiniPro12K())
        Screen_CodecURSAMiniPro12K(forceRefresh); // URSA Mini Pro 12K
      else
        DEBUG_DEBUG("No Codec screen for this camera.");
    } 
    else
      Screen_Codec4K6K(forceRefresh); // Handle no model name in 4K/6K screen
  }
  else
    Screen_Codec4K6K(forceRefresh); // If we don't have any codec info, we show the 4K/6K screen that shows no codec
}


// Resolution screen for Pocket 4K
void Screen_Resolution4K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Resolution;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Pocket 4K Resolution and Sensor Windows

  // Get the current Resolution and Codec
  CCUPacketTypes::RecordingFormatData currentRecordingFormat;
  if(camera->hasRecordingFormat())
    currentRecordingFormat = camera->getRecordingFormat();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present.
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Resolution4K: Btn A/B pressed");

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str(); // "HD", "2.6K 16:9", "4K DCI", "4K UHD", "HD"

    if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
    {
      int width = 0;
      int height = 0;
      bool window = false;

      if(currentRes == "4K DCI")
      {
        if(btnAPressed)
        {
          // 4K 2.4:1
          width = 4096; height = 1720;
          window = true;
        }
        else
        {
          // HD
          width = 1920; height = 1080;
          window = true;
        }
      }
      else if(currentRes == "4K 2.4:1")
      {
        if(btnAPressed)
        {
          // 4K UHD
          width = 3840; height = 2160;
          window = true;
        }
        else
        {
          // 4K DCI
          width = 4096; height = 2160;
        }
      }
      else if(currentRes == "4K UHD")
      {
        if(btnAPressed)
        {
          // 2.8K Ana
          width = 2880; height = 2160;
          window = true;
        }
        else
        {
          // 4K 2.4:1
          width = 4096; height = 1720;
          window = true;
        }
      }
      else if(currentRes == "2.8K Ana")
      {
        if(btnAPressed)
        {
          // 2.6K 16:9
          width = 2688; height = 1512;
          window = true;
        }
        else
        {
          // 4K UHD
          width = 3840; height = 2160;
          window = true;
        }
      }
      else if(currentRes == "2.6K 16:9")
      {
        if(btnAPressed)
        {
          // HD
          width = 1920; height = 1080;
          window = true;
        }
        else
        {
          // 2.8K Ana
          width = 2880; height = 2160;
          window = true;
        }
      }
      else // HD
      {
        if(btnAPressed)
        {
          // 4K DCI
          width = 4096; height = 2160;
        }
        else
        {
          // 2.6K 16:9
          width = 2688; height = 1512;
          window = true;
        }
      }

      if(width != 0)
      {
        // Resolution selected, write to camera
        CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
        newRecordingFormat.width = width;
        newRecordingFormat.height = height;
        newRecordingFormat.windowedModeEnabled = window;
        PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

        tappedAction = true;
      }
    }
    else
    {
      // ProRes

      int width = 0;
      int height = 0;
      bool window = false;

      DEBUG_DEBUG("Before change:");
      DEBUG_DEBUG(currentRecordingFormat.frameDimensionsShort_string().c_str());
      DEBUG_DEBUG(currentRecordingFormat.frameRate_string().c_str());
      DEBUG_DEBUG("Frame Rate: %hi", currentRecordingFormat.frameRate);
      DEBUG_DEBUG("Off Speed Frame Rate: %hi", currentRecordingFormat.offSpeedFrameRate);
      DEBUG_DEBUG("Width: %i", currentRecordingFormat.width);
      DEBUG_DEBUG("Height: %i", currentRecordingFormat.height);
      DEBUG_DEBUG("mRateEnabled: %s", currentRecordingFormat.mRateEnabled ? "Yes" : "No");
      DEBUG_DEBUG("offSpeedEnabled: %s", currentRecordingFormat.offSpeedEnabled ? "Yes" : "No");
      DEBUG_DEBUG("interlacedEnabled: %s", currentRecordingFormat.interlacedEnabled ? "Yes" : "No");
      DEBUG_DEBUG("windowedModeEnabled: %s", currentRecordingFormat.windowedModeEnabled ? "Yes" : "No");
      DEBUG_DEBUG("sensorMRateEnabled: %s", currentRecordingFormat.sensorMRateEnabled ? "Yes" : "No");

      if(currentRes == "4K DCI")
      {
        if(btnAPressed)
        {
          // 4K UHD
          width = 3840; height = 2160;
          window = true;
        }
        else
        {
          // HD
          width = 1920; height = 1080;
          window = true;
        }
      }
      else if(currentRes == "4K UHD")
      {
        if(btnAPressed)
        {
          // HD
          width = 1920; height = 1080;
          window = true;
        }
        else
        {
          // 4K DCI
          width = 4096; height = 2160;
          window = false;
        }
      }
      else // UHD
      {
        if(btnAPressed)
        {
          // 4K DCI
          width = 4096; height = 2160;
          window = false;
        }
        else
        {
          // 4K UHD
          width = 3840; height = 2160;
          window = true;
        }
      }

      if(width != 0)
      {
        // Resolution or Sensor Area selected, write to camera
        CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
        newRecordingFormat.width = width;
        newRecordingFormat.height = height;
        newRecordingFormat.windowedModeEnabled = window;

        DEBUG_DEBUG("Attempting to change to:");
        DEBUG_DEBUG(newRecordingFormat.frameDimensionsShort_string().c_str());
        DEBUG_DEBUG(newRecordingFormat.frameRate_string().c_str());
        DEBUG_DEBUG("Frame Rate: %hi", newRecordingFormat.frameRate);
        DEBUG_DEBUG("Off Speed Frame Rate: %hi", newRecordingFormat.offSpeedFrameRate);
        DEBUG_DEBUG("Width: %i", newRecordingFormat.width);
        DEBUG_DEBUG("Height: %i", newRecordingFormat.height);
        DEBUG_DEBUG("mRateEnabled: %s", newRecordingFormat.mRateEnabled ? "Yes" : "No");
        DEBUG_DEBUG("offSpeedEnabled: %s", newRecordingFormat.offSpeedEnabled ? "Yes" : "No");
        DEBUG_DEBUG("interlacedEnabled: %s", newRecordingFormat.interlacedEnabled ? "Yes" : "No");
        DEBUG_DEBUG("windowedModeEnabled: %s", newRecordingFormat.windowedModeEnabled ? "Yes" : "No");
        DEBUG_DEBUG("sensorMRateEnabled: %s", newRecordingFormat.sensorMRateEnabled ? "Yes" : "No");

        PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Resolution Pocket 4K Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
  {
    // Main label
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString("BRAW RESOLUTION", 30, 9, &AgencyFB_Bold9pt7b);

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str(); // "4K DCI", "4K 2.4:1", "4K UHD", "2.8K Ana", "2.6K 16:9", "HD"

    // 4K DCI
    String labelRes = "4K DCI";
    sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 65, 35);
    sprite->drawCentreString("DCI", 65, 58, &Lato_Regular5pt7b);

    // 4K 2.4:1
    labelRes = "4K 2.4:1";
    sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 160, 35);
    sprite->drawCentreString("2.4:1", 160, 58, &Lato_Regular5pt7b);

    // 4K UHD
    labelRes = "4K UHD";
    sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 260, 35);
    sprite->drawCentreString("UHD", 260, 58, &Lato_Regular5pt7b);

    // 2.8K Ana
    labelRes = "2.8K Ana";
    sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("2.8K", 65, 81);
    sprite->drawCentreString("ANAMORPHIC", 65, 104, &Lato_Regular5pt7b);

    // 2.6K 16:9
    labelRes = "2.6K 16:9";
    sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("2.6K", 160, 81);
    sprite->drawCentreString("16:9", 160, 104, &Lato_Regular5pt7b);

    // HD
    labelRes = "HD";
    sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("HD", 260, 87);

    // Sensor Area

    // Pocket 4K all are Windowed except 4K
    // sprite->drawSmoothRoundRect(20, 120, 5, 3, 290, 40, TFT_DARKGREY); // Optionally draw a rectangle around this info.
    sprite->drawCentreString(currentRes == "4K" ? "FULL SENSOR" :"SENSOR WINDOWED", 165, 129);
    sprite->drawCentreString(currentRecordingFormat.frameWidthHeight_string().c_str(), 165, 150, &Lato_Regular5pt7b);
  }
  else if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes)
  {
    // Main label
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString("ProRes RESOLUTION", 30, 9, &AgencyFB_Bold9pt7b);

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str(); // "4K DCI", "4K UHD", "HD"

    // 4K
    String labelRes = "4K DCI";
    sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 65, 36);
    sprite->drawCentreString("DCI", 65, 58, &Lato_Regular5pt7b);

    // 4K UHD
    labelRes = "4K UHD";
    sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("UHD", 160, 41);

    // HD
    labelRes = "HD";
    sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("HD", 260, 41);

    // Sensor Area

    // 4K DCI is Scaled from 5.7K, Ultra HD is Scaled from Full or 5.7K, HD is Scaled from Full, 5.7K, or 2.8K (however we can't change the 5.7K or 2.8K)

    sprite->drawString(currentRecordingFormat.frameWidthHeight_string().c_str(), 30, 90, &Lato_Regular5pt7b);
    sprite->drawString("SCALED / SENSOR AREA", 30, 105);

    if(currentRes == "4K DCI")
    {
      // Full sensor area only]
      sprite->fillSmoothRoundRect(20, 135, 90, 40, 3, TFT_DARKGREEN);
      sprite->drawCentreString("FULL", 65, 145);
    }
    else if(currentRes == "4K UHD")
    {
      // Full sensor area only]
      sprite->fillSmoothRoundRect(20, 135, 120, 40, 3, TFT_DARKGREEN);
      sprite->drawCentreString("WINDOW", 80, 145);
    }
    else
    {
      // HD

      // Scaled from Full, 2.6K or Windowed (however we can't tell which)
      sprite->fillSmoothRoundRect(20, 135, 290, 40, 3, TFT_DARKGREEN);
      sprite->drawCentreString("FULL / 2.6K / WINDOW", 165, 140);
      sprite->drawCentreString("CHECK/SET ON CAMERA", 165, 161, &Lato_Regular5pt7b);
    }
  }
  else
    DEBUG_ERROR("Resolution Pocket 4K - Codec not catered for.");

  sprite->pushSprite(0, 0);
}

// Resolution screen for Pocket 6K
void Screen_Resolution6K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Resolution;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Pocket 6K Resolution and Sensor Windows

  // Get the current Resolution and Codec
  CCUPacketTypes::RecordingFormatData currentRecordingFormat;
  if(camera->hasRecordingFormat())
    currentRecordingFormat = camera->getRecordingFormat();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present.
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Resolution6K: Btn A/B pressed");

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str();

    if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
    {
      int width = 0;
      int height = 0;
      bool window = false;

      if(currentRes == "6K")
      {
        if(btnAPressed)
        {
          // 6K 2.4:1
          width = 6144; height = 2560;
          window = true;
        }
        else
        {
          // 2.8K 17:9 - Note V8.1 width is 2880 not 2868
          width = 2880; height = 1512;
          window = true;
        }
      }
      else if(currentRes == "6K 2.4:1")
      {
        if(btnAPressed)
        {
          // 5.7K 17:9
          width = 5744; height = 3024;
          window = true;
        }
        else
        {
          // 6K
          width = 6144; height = 3456;
        }
      }
      else if(currentRes == "5.7K 17:9")
      {
        if(btnAPressed)
        {
          // 4K DCI
          width = 4096; height = 2160;
          window = true;
        }
        else
        {
          // 6K 2.4:1
          width = 6144; height = 2560;
          window = true;
        }
      }
      else if(currentRes == "4K DCI")
      {
        if(btnAPressed)
        {
          // 3.7K Anamorphic
          width = 3728; height = 3104;
          window = true;
        }
        else
        {
          // 5.7K 17:9
          width = 5744; height = 3024;
          window = true;
        }
      }
      else if(currentRes == "3.7K 6:5A")
      {
        if(btnAPressed)
        {
          // 2.8K 17:9 - Note V8.1 width is 2880 not 2868
          width = 2880; height = 1512;
          window = true;
        }
        else
        {
          // 4K DCI
          width = 4096; height = 2160;
          window = true;
        }
      }
      else // 2.8K 17:9
      {
        if(btnAPressed)
        {
          // 6K
          width = 6144; height = 3456;
        }
        else
        {
          // 3.7K Anamorphic
          width = 3728; height = 3104;
          window = true;
        }
      }

      if(width != 0)
      {
        // Resolution selected, write to camera
        CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
        newRecordingFormat.width = width;
        newRecordingFormat.height = height;
        newRecordingFormat.windowedModeEnabled = window;
        PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

        tappedAction = true;
      }
    }
    else
    {
      // ProRes

      int width = 0;
      int height = 0;
      bool window = false;

      DEBUG_DEBUG("Before change:");
      DEBUG_DEBUG(currentRecordingFormat.frameDimensionsShort_string().c_str());
      DEBUG_DEBUG(currentRecordingFormat.frameRate_string().c_str());
      DEBUG_DEBUG("Frame Rate: %hi", currentRecordingFormat.frameRate);
      DEBUG_DEBUG("Off Speed Frame Rate: %hi", currentRecordingFormat.offSpeedFrameRate);
      DEBUG_DEBUG("Width: %i", currentRecordingFormat.width);
      DEBUG_DEBUG("Height: %i", currentRecordingFormat.height);
      DEBUG_DEBUG("mRateEnabled: %s", currentRecordingFormat.mRateEnabled ? "Yes" : "No");
      DEBUG_DEBUG("offSpeedEnabled: %s", currentRecordingFormat.offSpeedEnabled ? "Yes" : "No");
      DEBUG_DEBUG("interlacedEnabled: %s", currentRecordingFormat.interlacedEnabled ? "Yes" : "No");
      DEBUG_DEBUG("windowedModeEnabled: %s", currentRecordingFormat.windowedModeEnabled ? "Yes" : "No");
      DEBUG_DEBUG("sensorMRateEnabled: %s", currentRecordingFormat.sensorMRateEnabled ? "Yes" : "No");

      if(currentRes == "4K DCI")
      {
        if(btnAPressed)
        {
          // 4K UHD
          width = 3840; height = 2160;
          window = false;
        }
        else
        {
          // HD
          width = 1920; height = 1080;
          window = false;
        }
      }
      else if(currentRes == "4K UHD")
      {
        if(btnAPressed)
        {
          // HD
          width = 1920; height = 1080;
          window = false;
        }
        else
        {
          // 4K DCI
          width = 4096; height = 2160;
          window = true;
        }
      }
      else // UHD
      {
        if(btnAPressed)
        {
          // 4K DCI
          width = 4096; height = 2160;
          window = true;
        }
        else
        {
          // 4K UHD
          width = 3840; height = 2160;
          window = false;
        }
      }

      if(width != 0)
      {
        // Resolution or Sensor Area selected, write to camera
        CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
        newRecordingFormat.width = width;
        newRecordingFormat.height = height;
        newRecordingFormat.windowedModeEnabled = window;

        DEBUG_DEBUG("Attempting to change to:");
        DEBUG_DEBUG(newRecordingFormat.frameDimensionsShort_string().c_str());
        DEBUG_DEBUG(newRecordingFormat.frameRate_string().c_str());
        DEBUG_DEBUG("Frame Rate: %hi", newRecordingFormat.frameRate);
        DEBUG_DEBUG("Off Speed Frame Rate: %hi", newRecordingFormat.offSpeedFrameRate);
        DEBUG_DEBUG("Width: %i", newRecordingFormat.width);
        DEBUG_DEBUG("Height: %i", newRecordingFormat.height);
        DEBUG_DEBUG("mRateEnabled: %s", newRecordingFormat.mRateEnabled ? "Yes" : "No");
        DEBUG_DEBUG("offSpeedEnabled: %s", newRecordingFormat.offSpeedEnabled ? "Yes" : "No");
        DEBUG_DEBUG("interlacedEnabled: %s", newRecordingFormat.interlacedEnabled ? "Yes" : "No");
        DEBUG_DEBUG("windowedModeEnabled: %s", newRecordingFormat.windowedModeEnabled ? "Yes" : "No");
        DEBUG_DEBUG("sensorMRateEnabled: %s", newRecordingFormat.sensorMRateEnabled ? "Yes" : "No");

        PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Resolution Pocket 6K Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
  {
    // Main label
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString("BRAW RESOLUTION", 30, 9);

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str(); // "6K", "6K 2.4:1", "5.7K 17:9", "4K DCI", "3.7K 6:5A", "2.8K 17:9"

    // 6K
    String labelRes = "6K";
    sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(String(labelRes).c_str(), 65, 41);

    // 6K, 2.4:1
    labelRes = "6K 2.4:1";
    sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("6K", 160, 36);
    sprite->drawCentreString("2.4:1", 160, 58, &Lato_Regular5pt7b);
  
    // 5.7K, 17:9
    labelRes = "5.7K 17:9";
    sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("5.7K", 260, 36);
    sprite->drawCentreString("17:9", 260, 58, &Lato_Regular5pt7b);

    // 4K DCI
    labelRes = "4K DCI";
    sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 65, 82);
    sprite->drawCentreString("DCI", 65, 104, &Lato_Regular5pt7b);

    // 3.7K 6:5A
    labelRes = "3.7K 6:5A";
    sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("3.7K", 160, 82);
    sprite->drawCentreString("6:5 ANA", 160, 104, &Lato_Regular5pt7b);

    // 2.8K 17:9
    labelRes = "2.8K 17:9";
    sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("2.8K", 260, 82);
    sprite->drawCentreString("17:9", 260, 104, &Lato_Regular5pt7b);

    // Sensor Area

    // Pocket 6K all are Windowed except 6K
    // sprite->drawSmoothRoundRect(20, 120, 5, 3, 290, 40, TFT_DARKGREY); // Optionally draw a rectangle around this info.
    sprite->drawCentreString(currentRes == "6K" ? "FULL SENSOR" :"SENSOR WINDOWED", 165, 129);
    sprite->drawCentreString(currentRecordingFormat.frameWidthHeight_string().c_str(), 165, 148, &Lato_Regular5pt7b);
  }
  else if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes)
  {
    // Main label
    sprite->setTextColor(TFT_WHITE);
    sprite->drawString("ProRes RESOLUTION", 30, 9);

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str(); // "4K DCI", "4K UHD", "HD"

    // 4K
    String labelRes = "4K DCI";
    sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 65, 36);
    sprite->drawCentreString("DCI", 65, 58, &Lato_Regular5pt7b);

    // 4K UHD
    labelRes = "4K UHD";
    sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("UHD", 160, 41);

    // HD
    labelRes = "HD";
    sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("HD", 260, 41);

    // Sensor Area

    // 4K DCI is Scaled from 5.7K, Ultra HD is Scaled from Full or 5.7K, HD is Scaled from Full, 5.7K, or 2.8K (however we can't change the 5.7K or 2.8K)

    sprite->drawString(currentRecordingFormat.frameWidthHeight_string().c_str(), 30, 90, &Lato_Regular5pt7b);
    sprite->drawString("SCALED FROM SENSOR AREA", 30, 105);

    if(currentRes == "4K DCI")
    {
      // Full sensor area only]
      sprite->fillSmoothRoundRect(20, 130, 90, 40, 3, TFT_DARKGREEN);
      sprite->drawCentreString("FULL", 65, 141);
    }
    else if(currentRes == "4K UHD")
    {
      // Scaled from Full or 5.7K
      sprite->fillSmoothRoundRect(20, 130, 90, 40, 3, (!currentRecordingFormat.windowedModeEnabled ? TFT_DARKGREEN : TFT_DARKGREY));
      sprite->drawCentreString("FULL", 65, 141);

      // 5.7K
      sprite->fillSmoothRoundRect(115, 130, 90, 40, 3, (currentRecordingFormat.windowedModeEnabled ? TFT_DARKGREEN : TFT_DARKGREY));
      sprite->drawCentreString("5.3K", 160, 141);
    }
    else
    {
      // HD

      // Scaled from Full, 2.8K or 5.7K (however we can't tell if it's 2.8K or 5.7K)
      sprite->fillSmoothRoundRect(20, 130, 90, 40, 3, (!currentRecordingFormat.windowedModeEnabled ? TFT_DARKGREEN : TFT_DARKGREY));
      sprite->drawCentreString("FULL", 65, 141);

      // 2.8K 5.7K
      sprite->fillSmoothRoundRect(115, 130, 195, 40, 3, (currentRecordingFormat.windowedModeEnabled ? TFT_DARKGREEN : TFT_DARKGREY));
      sprite->drawCentreString("5.3K / 2.8K", 212, 136);
      sprite->drawCentreString("CHECK/SET ON CAMERA", 212, 156, &Lato_Regular5pt7b);
    }
  }
  else
    DEBUG_ERROR("Resolution Pocket 6K - Codec not catered for.");

  sprite->pushSprite(0, 0);
}

// Resolution Screen for URSA Mini Pro G2
void Screen_ResolutionURSAMiniProG2(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Resolution;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // URSA Mini Pro G2 (no window settings, just resolution)

  // Get the current Resolution and Codec
  CCUPacketTypes::RecordingFormatData currentRecordingFormat;
  if(camera->hasRecordingFormat())
    currentRecordingFormat = camera->getRecordingFormat();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present.
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("ResolutionURSAMiniProG2: Btn A/B pressed");

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str();
    int currentWidth = currentRecordingFormat.width;
    int currentHeight = currentRecordingFormat.height;

    // URSA Mini Pro G2 has the same resolutions for BRAW and ProRes
    if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW || currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes)
    {
      int width = 0;
      int height = 0;
      bool window = true;

      if(currentRes == "4.6K")
      {
        if(btnAPressed)
        {
          // 4.6K 2.4:1
          width = 4608; height = 1920;
        }
        else
        {
          // HD
          width = 1920; height = 1080;
        }
      }
      else if(currentRes == "4.6K 2.4:1")
      {
        if(btnAPressed)
        {
          // 4K 16:9
          width = 4096; height = 2304;
        }
        else
        {
          // 4.6K
          width = 4608; height = 2592;
          window = false;
        }
      }
      else if(currentRes == "4K 16:9")
      {
        if(btnAPressed)
        {
          // 4K DCI
          width = 4096; height = 2160;
        }
        else
        {
          // 4.6K 2.4:1
          width = 4608; height = 1920;
        }
      }
      else if(currentRes == "4K DCI")
      {
        if(btnAPressed)
        {
          // 4K UHD
          width = 3840; height = 2160;
        }
        else
        {
          // 4K 16:9
          width = 4096; height = 2304;
        }
      }
      else if(currentRes == "4K UHD")
      {
        if(btnAPressed)
        {
          // 3K Ana
          width = 3072; height = 2560;
        }
        else
        {
          // 4K DCI
          width = 4096; height = 2160;
        }
      }
      else if(currentRes == "3K Ana")
      {
        if(btnAPressed)
        {
          // 2K 16:9
          width = 2048; height = 1152;
        }
        else
        {
          // 4K UHD
          width = 3840; height = 2160;
        }
      }
      else if(currentRes == "2K 16:9")
      {
        if(btnAPressed)
        {
          // 2K DCI
          width = 2048; height = 1080;
        }
        else
        {
          // 3K Ana
          width = 3072; height = 2560;
        }
      }
      else if(currentRes == "2K DCI")
      {
        if(btnAPressed)
        {
          // HD
          width = 1920; height = 1080;
        }
        else
        {
          // 2K 16:9
          width = 2048; height = 1152;
        }
      }
      else // HD
      {
        if(btnAPressed)
        {
          // 4.6K
          width = 4608; height = 2592;
          window = false;
        }
        else
        {
          // 2K DCI
          width = 2048; height = 1080;
        }
      }

      if(width != 0)
      {
        // Resolution selected, write to camera
        CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
        newRecordingFormat.width = width;
        newRecordingFormat.height = height;
        newRecordingFormat.windowedModeEnabled = window;
        PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

        tappedAction = true;
      }
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Resolution URSA Mini Pro G2 Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW || currentCodec.basicCodec == CCUPacketTypes::BasicCodec::ProRes)
  {
    // Main label
    sprite->setTextColor(TFT_WHITE);
    if(currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW)
      sprite->drawString("BRAW RESOLUTION", 30, 9, &AgencyFB_Bold9pt7b);
    else
      sprite->drawString("PRORES RESOLUTION", 30, 9, &AgencyFB_Bold9pt7b);

    String currentRes = currentRecordingFormat.frameDimensionsShort_string().c_str(); // "4.6K", "4.6K 2.4:1", "4K 16:9", "4K DCI", "4K UHD", "3K Ana", "2K 16:9", "2K DCI", "HD"

    // 4.6K
    String labelRes = "4.6K";
    sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(String(labelRes).c_str(), 65, 41);

    // 4.6K 2.4:1
    labelRes = "4.6K 2.4:1";
    sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4.6K", 160, 36);
    sprite->drawCentreString("2.4:1", 160, 58, &Lato_Regular5pt7b);
  
    // 4K 16:9
    labelRes = "4K 16:9";
    sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4.6K", 260, 36);
    sprite->drawCentreString("16:9", 260, 58, &Lato_Regular5pt7b);

    // 4K DCI
    labelRes = "4K DCI";
    sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("4K", 65, 82);
    sprite->drawCentreString("DCI", 65, 104, &Lato_Regular5pt7b);

    // 4K UHD
    labelRes = "4K UHD";
    sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("UHD", 160, 86);

    // 3K Ana
    labelRes = "3K Ana";
    sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("3K", 260, 82);
    sprite->drawCentreString("ANA", 260, 104, &Lato_Regular5pt7b);

    // 2K 16:9
    labelRes = "2K 16:9";
    sprite->fillSmoothRoundRect(20, 120, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("2K", 65, 128);
    sprite->drawCentreString("16:9", 65, 149, &Lato_Regular5pt7b);

    // 2K DCI
    labelRes = "2K DCI";
    sprite->fillSmoothRoundRect(115, 120, 90, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("2K", 160, 128);
    sprite->drawCentreString("DCI", 160, 149, &Lato_Regular5pt7b);

    // HD
    labelRes = "HD";
    sprite->fillSmoothRoundRect(210, 120, 100, 40, 3, (currentRes == labelRes ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString("HD", 260, 132);
  }
  else
    DEBUG_ERROR("Resolution URSA Mini Pro G2 - Codec not catered for.");

  sprite->pushSprite(0, 0);
}

// Resolution Screen for URSA Mini Pro 12K
void Screen_ResolutionURSAMiniPro12K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Resolution;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  
  // TO DO
}

// Resolution screen - redirects to appropriate screen for camera
void Screen_Resolution(bool forceRefresh = false)
{
  auto camera = BMDControlSystem::getInstance()->getCamera();

  if(camera->hasCodec())
  {
    if(camera->hasRecordingFormat())
    {
      if(camera->isPocket4K())
        Screen_Resolution4K(forceRefresh); // Pocket 4K
      else if(camera->isPocket6K())
        Screen_Resolution6K(forceRefresh); // Pocket 6K
      else if(camera->isURSAMiniProG2())
        Screen_ResolutionURSAMiniProG2(forceRefresh); // URSA Mini Pro G2
      else if(camera->isURSAMiniPro12K())
        Screen_ResolutionURSAMiniPro12K(forceRefresh); // URSA Mini Pro 12K
      else
        DEBUG_DEBUG("No Codec screen for this camera.");
    } 
    else
      Screen_Resolution4K(forceRefresh); // Handle no model name in 4K screen
  }
  else
    Screen_Resolution4K(forceRefresh); // If we don't have any codec info, we show the 4K screen that shows no codec
}

// Frame Rate screen for Pocket 4K
void Screen_Framerate4K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Framerate;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Pocket 4K Frame Rate

  // Get the current Resolution and Codec
  CCUPacketTypes::RecordingFormatData currentRecordingFormat;
  if(camera->hasRecordingFormat())
    currentRecordingFormat = camera->getRecordingFormat();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present.
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  std::string currentFrameRate = currentRecordingFormat.frameRate_string();

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("FrameRate4K: Btn A/B pressed");

    short frameRate = 0;
    bool mRateEnabled = false;

    if(currentFrameRate == "23.98")
    {
      if(btnBPressed)
      {
        // 24
        frameRate = 24;
      }
      else
      {
        // 60
        frameRate = 60;
      }
    }
    else if(currentFrameRate == "24")
    {
      if(btnBPressed)
      {
        // 25
        frameRate = 25;
      }
      else
      {
        // 23.98
        frameRate = 24;
        mRateEnabled = true;
      }
    }
    else if(currentFrameRate == "25")
    {
      if(btnBPressed)
      {
        // 29.97
        frameRate = 30;
        mRateEnabled = true;
      }
      else
      {
        // 24
        frameRate = 24;
      }
    }
    else if(currentFrameRate == "29.97")
    {
      if(btnBPressed)
      {
        // 30
        frameRate = 30;
      }
      else
      {
        // 25
        frameRate = 25;
      }
    }
    else if(currentFrameRate == "30")
    {
      if(btnBPressed)
      {
        // 50
        frameRate = 50;
      }
      else
      {
        // 30
        frameRate = 30;
        mRateEnabled = true;
      }
    }
    else if(currentFrameRate == "50")
    {
      if(btnBPressed)
      {
        // 59.94
        frameRate = 60;
        mRateEnabled = true;
      }
      else
      {
        // 30
        frameRate = 30;
      }
    }
    else if(currentFrameRate == "59.94")
    {
      if(btnBPressed)
      {
        // 60
        frameRate = 60;
      }
      else
      {
        // 50
        frameRate = 50;
      }
    }
    else if(currentFrameRate == "60")
    {
      if(btnBPressed)
      {
        // 23.98
        frameRate = 24;
        mRateEnabled = true;
      }
      else
      {
        // 59.94
        frameRate = 60;
        mRateEnabled = true;
      }
    }

    if(frameRate != 0)
    {
      // Frame rate selected, write to camera
      CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
      newRecordingFormat.frameRate = frameRate;
      newRecordingFormat.mRateEnabled = mRateEnabled;
      PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

      tappedAction = true;
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Frame Rate Pocket 4K Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // Main label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("FRAME RATE", 30, 9, &AgencyFB_Bold9pt7b);

  // Output the current Codec and Resolution
  String codecRes = currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW ? "BRAW | " :"ProRes | ";
  codecRes.concat(currentRecordingFormat.frameDimensionsShort_string().c_str());
  sprite->drawString(codecRes, 30, 167);

  sprite->drawString(currentRecordingFormat.frameWidthHeight_string().c_str(), 30, 189, &Lato_Regular5pt7b);

  // 23.98
  std::string labelFR = "23.98";
  sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 65, 41);

  // 24
  labelFR = "24";
  sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 160, 41);

  // 25
  labelFR = "25";
  sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 260, 41);

  // 29.97
  labelFR = "29.97";
  sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 65, 87);

  // 30
  labelFR = "30";
  sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 160, 87);

  // 50
  labelFR = "50";
  sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 260, 87);

  // 59.94
  labelFR = "59.94";
  sprite->fillSmoothRoundRect(20, 120, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 65, 131);

  // 60
  labelFR = "60";
  sprite->fillSmoothRoundRect(115, 120, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 160, 131);

  sprite->pushSprite(0, 0);
}

// Frame Rate Screen for Pocket 6K
void Screen_Framerate6K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Framerate;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // Pocket 6K Frame Rate

  // Get the current Resolution and Codec
  CCUPacketTypes::RecordingFormatData currentRecordingFormat;
  if(camera->hasRecordingFormat())
    currentRecordingFormat = camera->getRecordingFormat();

  // Get the current Codec values. getCodec() throws if the camera hasn't sent
  // a codec yet, so only fetch it when one is present.
  CodecInfo currentCodec(CCUPacketTypes::BasicCodec::BRAW, CCUPacketTypes::CodecVariants::kDefault);
  if(camera->hasCodec())
    currentCodec = camera->getCodec();

  std::string currentFrameRate = currentRecordingFormat.frameRate_string();

  bool isFull6K = currentRecordingFormat.width == 6144 && currentRecordingFormat.height == 3456; // Are we in full 6K (only goes up to 50fps)

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("FrameRate6K: Btn A/B pressed");

    short frameRate = 0;
    bool mRateEnabled = false;

    if(currentFrameRate == "23.98")
    {
      if(btnBPressed)
      {
        // 24
        frameRate = 24;
      }
      else
      {
        if(isFull6K)
          frameRate = 50;
        else
          frameRate = 60;
      }
    }
    else if(currentFrameRate == "24")
    {
      if(btnBPressed)
      {
        // 25
        frameRate = 25;
      }
      else
      {
        // 23.98
        frameRate = 24;
        mRateEnabled = true;
      }
    }
    else if(currentFrameRate == "25")
    {
      if(btnBPressed)
      {
        // 29.97
        frameRate = 30;
        mRateEnabled = true;
      }
      else
      {
        // 24
        frameRate = 24;
      }
    }
    else if(currentFrameRate == "29.97")
    {
      if(btnBPressed)
      {
        // 30
        frameRate = 30;
      }
      else
      {
        // 25
        frameRate = 25;
      }
    }
    else if(currentFrameRate == "30")
    {
      if(btnBPressed)
      {
        // 50
        frameRate = 50;
      }
      else
      {
        // 30
        frameRate = 30;
        mRateEnabled = true;
      }
    }
    else if(currentFrameRate == "50")
    {
      if(btnBPressed)
      {
        if(isFull6K)
        {
          // 23.98
          frameRate = 24;
          mRateEnabled = true;
        }
        else
        {
          // 59.94
          frameRate = 60;
          mRateEnabled = true;
        }
      }
      else
      {
        // 30
        frameRate = 30;
      }
    }
    else if(currentFrameRate == "59.94")
    {
      if(btnBPressed)
      {
        // 60
        frameRate = 60;
      }
      else
      {
        // 50
        frameRate = 50;
      }
    }
    else if(currentFrameRate == "60")
    {
      if(btnBPressed)
      {
        // 23.98
        frameRate = 24;
        mRateEnabled = true;
      }
      else
      {
        // 59.94
        frameRate = 60;
        mRateEnabled = true;
      }
    }

    if(frameRate != 0)
    {
      // Frame rate selected, write to camera
      CCUPacketTypes::RecordingFormatData newRecordingFormat = currentRecordingFormat;
      newRecordingFormat.frameRate = frameRate;
      newRecordingFormat.mRateEnabled = mRateEnabled;
      PacketWriter::writeRecordingFormat(newRecordingFormat, &cameraConnection);

      tappedAction = true;
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Frame Rate Pocket 4K Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

  // Main label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("FRAME RATE", 30, 9, &AgencyFB_Bold9pt7b);

  // Output the current Codec and Resolution
  String codecRes = currentCodec.basicCodec == CCUPacketTypes::BasicCodec::BRAW ? "BRAW | " :"ProRes | ";
  codecRes.concat(currentRecordingFormat.frameDimensionsShort_string().c_str());
  sprite->drawString(codecRes, 30, 167);

  sprite->drawString(currentRecordingFormat.frameWidthHeight_string().c_str(), 30, 189, &Lato_Regular5pt7b);

  // 23.98
  std::string labelFR = "23.98";
  sprite->fillSmoothRoundRect(20, 30, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 65, 41);

  // 24
  labelFR = "24";
  sprite->fillSmoothRoundRect(115, 30, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 160, 41);

  // 25
  labelFR = "25";
  sprite->fillSmoothRoundRect(210, 30, 100, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 260, 41);

  // 29.97
  labelFR = "29.97";
  sprite->fillSmoothRoundRect(20, 75, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 65, 87);

  // 30
  labelFR = "30";
  sprite->fillSmoothRoundRect(115, 75, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 160, 87);

  // 50
  labelFR = "50";
  sprite->fillSmoothRoundRect(210, 75, 100, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawCentreString(labelFR.c_str(), 260, 87);

  if(!isFull6K)
  {
    // 59.94
    labelFR = "59.94";
    sprite->fillSmoothRoundRect(20, 120, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(labelFR.c_str(), 65, 131);

    // 60
    labelFR = "60";
    sprite->fillSmoothRoundRect(115, 120, 90, 40, 3, (currentFrameRate == labelFR ? TFT_DARKGREEN : TFT_DARKGREY));
    sprite->drawCentreString(labelFR.c_str(), 160, 131);
  }

  sprite->pushSprite(0, 0);
}

// Frame Rate Screen for URSA Mini Pro G2
void Screen_FramerateURSAMiniProG2(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Resolution;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  
  // TO DO
}

// Frame Rate Screen for URSA Mini Pro 12K
void Screen_FramerateURSAMiniPro12K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Resolution;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  
  // TO DO
}

// Frame Rate screen - redirects to appropriate screen for camera
void Screen_Framerate(bool forceRefresh = false)
{
  auto camera = BMDControlSystem::getInstance()->getCamera();

  if(camera->hasCodec())
  {
    if(camera->hasRecordingFormat())
    {
      if(camera->isPocket4K())
        Screen_Framerate4K(forceRefresh); // Pocket 4K
      else if(camera->isPocket6K())
        Screen_Framerate6K(forceRefresh); // Pocket 6K
      else if(camera->isURSAMiniProG2())
        Screen_FramerateURSAMiniProG2(forceRefresh); // URSA Mini Pro G2
      else if(camera->isURSAMiniPro12K())
        Screen_FramerateURSAMiniPro12K(forceRefresh); // URSA Mini Pro 12K
      else
        DEBUG_DEBUG("No Codec screen for this camera.");
    } 
    else
      Screen_Framerate4K(forceRefresh); // Handle no model name in 4K screen
  }
  else
    Screen_Framerate4K(forceRefresh); // If we don't have any codec info, we show the 4K screen that shows no codec
}


// Media screen for Pocket 4K
void Screen_Media4K6K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Media;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  // 3 Media Slots - CFAST, SD, USB

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Media Pocket 4K/6K: Btn A/B pressed");

    // Make sure we have some media
    bool hasMedia = camera->getMediaSlotSafe(0).status != CCUPacketTypes::MediaStatus::None || camera->getMediaSlotSafe(1).status != CCUPacketTypes::MediaStatus::None || camera->getMediaSlotSafe(2).status != CCUPacketTypes::MediaStatus::None;

    if(hasMedia)
    {
      TransportInfo transportInfo = camera->getTransportMode();

      bool slotAvail1 = camera->getMediaSlotSafe(0).status != CCUPacketTypes::MediaStatus::None;
      bool slotAvail2 = camera->getMediaSlotSafe(1).status != CCUPacketTypes::MediaStatus::None;
      bool slotAvail3 = camera->getMediaSlotSafe(2).status != CCUPacketTypes::MediaStatus::None;

      // Make sure we have more than one available slot
      if(slotAvail1 + slotAvail2 + slotAvail3 > 1)
      {
        short changeToSlot = -1;
        short slotActive = 0;
        if(slotAvail1 && transportInfo.slots[0].active)
          slotActive = 1;
        else if(slotAvail2 && transportInfo.slots[1].active)
          slotActive = 2;
        else
          slotActive = 3;

        if(btnAPressed)
        {
          // Down to the next
          switch(slotActive)
          {
            case 1:
              changeToSlot = slotAvail2 ? 2 : 3;
              break;
            case 2:
              changeToSlot = slotAvail3 ? 3 : 1;
              break;
            case 3:
              changeToSlot = slotAvail1 ? 1 : 2;
              break;
          }
        }
        else
        {
          // Up to the previous
          switch(slotActive)
          {
            case 1:
              changeToSlot = slotAvail3 ? 3 : 2;
              break;
            case 2:
              changeToSlot = slotAvail1 ? 1 : 3;
              break;
            case 3:
              changeToSlot = slotAvail2 ? 2 : 1;
              break;
          }
        }

        // Change the media slot
        if(changeToSlot != -1)
        {
          transportInfo.slots[0].active = changeToSlot == 1 ? true : false;
          transportInfo.slots[1].active = changeToSlot == 2 ? true : false;
          transportInfo.slots[2].active = changeToSlot == 3 ? true : false;
          PacketWriter::writeTransportInfo(transportInfo, &cameraConnection);

          tappedAction = true;
        }
      }
      else
        DEBUG_DEBUG("Only 1 media available, can't change active media.");
    }
    else
      DEBUG_DEBUG("No media, can't change active media.");
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();
  
  DEBUG_DEBUG("Screen Media Pocket 4K/6K Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

    // Media label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("MEDIA", 30, 9, &AgencyFB_Bold9pt7b);

  // CFAST
  BMDCamera::MediaSlot cfast = camera->getMediaSlotSafe(0);
  sprite->fillSmoothRoundRect(20, 30, 295, 40, 3, (cfast.active ? TFT_DARKGREEN : TFT_DARKGREY));
  if(cfast.StatusIsError()) sprite->drawRoundRect(20, 30, 295, 40, 3, (cfast.active ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawString("CFAST", 28, 47);
  if(cfast.status != CCUPacketTypes::MediaStatus::None) sprite->drawString(cfast.remainingRecordTimeString.c_str(), 155, 47);

  if(cfast.status != CCUPacketTypes::MediaStatus::None) sprite->drawString("REMAINING TIME", 155, 35, &Lato_Regular5pt7b);
  sprite->drawString("1", 300, 35, &Lato_Regular5pt7b);
  sprite->drawString(cfast.GetStatusString().c_str(), 28, 35, &Lato_Regular5pt7b);

  // SD
  BMDCamera::MediaSlot sd = camera->getMediaSlotSafe(1);
  sprite->fillSmoothRoundRect(20, 75, 295, 40, 3, (sd.active ? TFT_DARKGREEN : TFT_DARKGREY));
  if(sd.StatusIsError()) sprite->drawRoundRect(20, 75, 295, 40, 3, (sd.active ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawString("SD", 28, 92);
  if(sd.status != CCUPacketTypes::MediaStatus::None) sprite->drawString(sd.remainingRecordTimeString.c_str(), 155, 92);

  if(sd.status != CCUPacketTypes::MediaStatus::None) sprite->drawString("REMAINING TIME", 155, 80, &Lato_Regular5pt7b);
  sprite->drawString("2", 300, 80, &Lato_Regular5pt7b);
  sprite->drawString(sd.GetStatusString().c_str(), 28, 80, &Lato_Regular5pt7b);
  if(sd.StatusIsError()) sprite->setTextColor(TFT_WHITE);

  // USB
  BMDCamera::MediaSlot usb = camera->getMediaSlotSafe(2);
  sprite->fillSmoothRoundRect(20, 120, 295, 40, 3, (usb.active ? TFT_DARKGREEN : TFT_DARKGREY));
  if(usb.StatusIsError()) sprite->drawRoundRect(20, 120, 295, 40, 3, (usb.active ? TFT_DARKGREEN : TFT_DARKGREY));
  sprite->drawString("USB", 28, 137);
  if(usb.status != CCUPacketTypes::MediaStatus::None) sprite->drawString(usb.remainingRecordTimeString.c_str(), 155, 137);
  
  if(usb.status != CCUPacketTypes::MediaStatus::None) sprite->drawString("REMAINING TIME", 155, 125, &Lato_Regular5pt7b);
  sprite->drawString("3", 300, 125, &Lato_Regular5pt7b);
  sprite->drawString(usb.GetStatusString().c_str(), 28, 125, &Lato_Regular5pt7b);
  if(usb.StatusIsError()) sprite->setTextColor(TFT_WHITE);

  sprite->pushSprite(0, 0);
}

// Media Screen for URSA Mini Pro G2
void Screen_MediaURSAMiniProG2(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Media;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  
  // TO DO
}

// Media Screen for URSA Mini Pro 12K
void Screen_MediaURSAMiniPro12K(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Media;

  auto camera = BMDControlSystem::getInstance()->getCamera();
  
  // TO DO
}


// Media screen - redirects to appropriate screen for camera
void Screen_Media(bool forceRefresh = false)
{
  auto camera = BMDControlSystem::getInstance()->getCamera();

  if(camera->getMediaSlots().size() != 0)
  {
    if(camera->hasModelName())
    {
      if(camera->isPocket4K6K())
        Screen_Media4K6K(forceRefresh); // Pocket camera
      else if(camera->isURSAMiniProG2())
        Screen_MediaURSAMiniProG2(forceRefresh); // URSA Mini Pro G2
      else if(camera->isURSAMiniPro12K())
        Screen_MediaURSAMiniPro12K(forceRefresh); // URSA Mini Pro 12K
      else
        DEBUG_DEBUG("No Media screen for this camera.");
    } 
    else
      Screen_Media4K6K(forceRefresh); // Handle no model name in 4K/6K screen
  }
  else
    Screen_Media4K6K(forceRefresh); // If we don't have any media info, we show the 4K/6K screen that shows no media

}

void Screen_Lens(bool forceRefresh = false)
{
  if(!BMDControlSystem::getInstance()->hasCamera())
    return;

  connectedScreenIndex = Screens::Lens;

  auto camera = BMDControlSystem::getInstance()->getCamera();

  bool tappedAction = false;
  if(btnAPressed || btnBPressed)
  {
    DEBUG_DEBUG("Lens: Btn A/B pressed");

    if(btnAPressed)
    {
      // Focus button
      PacketWriter::writeAutoFocus(&cameraConnection);

      DEBUG_DEBUG("Instantaneous Autofocus");
      
      tappedAction = true;
    }
  }

  // If the screen hasn't changed, there were no touch events and we don't have to refresh, return.
  if(lastRefreshedScreen == camera->getLastModified() && !forceRefresh && !tappedAction)
    return;
  else
    lastRefreshedScreen = camera->getLastModified();

  DEBUG_DEBUG("Screen Lens Refreshed.");

  sprite->fillScreen(TFT_BLACK);

  Screen_Common_Connected(); // Common elements

    // Media label
  sprite->setTextColor(TFT_WHITE);
  sprite->drawString("LENS", 30, 9, &AgencyFB_Bold9pt7b);

  // M5GFX, set font here rather than on each drawString line
  sprite->setFont(&Lato_Regular11pt7b);

  sprite->drawString("LENS TYPE", 30, 53, &Lato_Regular5pt7b);
  if(camera->hasLensType())
  {
    sprite->setTextColor(TFT_LIGHTGREY);
    sprite->drawString(camera->getLensType().c_str(), 30, 35, &Lato_Regular6pt7b);

  }
  
  sprite->drawString("FOCAL LENGTH", 30, 98, &Lato_Regular5pt7b);
  if(camera->hasFocalLengthMM() || camera->hasLensFocalLength())
  {
    if(camera->hasFocalLengthMM())
    {
      auto focalLength = camera->getFocalLengthMM();
      std::string focalLengthMM = std::to_string(focalLength);
      std::string combined = focalLengthMM + "mm";

      sprite->drawString(combined.c_str(), 30, 75, &Lato_Regular12pt7b);
    }
    else
      sprite->drawString(camera->getLensFocalLength().c_str(), 30, 80, &Lato_Regular12pt7b);
  }

  sprite->drawString("LENS DISTANCE", 30, 143, &Lato_Regular5pt7b);
  if(camera->hasLensDistance())
  {
    sprite->drawString(camera->getLensDistance().c_str(), 30, 120, &Lato_Regular12pt7b);
  }

  sprite->drawString("APERTURE", 30, 188, &Lato_Regular5pt7b);
  if(camera->hasApertureFStopString())
  {
    sprite->drawString(camera->getApertureFStopString().c_str(), 30, 165, &Lato_Regular12pt7b);
  }

  sprite->pushSprite(0, 0);
}

// Bump the MPU6886 output data rate from M5Unified's default 500 Hz to
// 1 kHz. M5Unified's MPU6886_Class::begin() leaves SMPLRT_DIV = 0x03
// (1/4 divider -> 500 Hz) with CONFIG = 0x01 (44 Hz DLPF). Writing
// SMPLRT_DIV = 0x00 (no divider) raises the ODR to the sensor's native
// 1 kHz, which is what the GCSV logger needs. This is the same trick as
// the Hackaday "Good Vibrations" log: only the sample-rate divider is
// changed, everything else (DLPF, FSR) stays as M5Unified configured it.
// The GCSV timestamps come from the camera timecode, not the IMU, so a
// faster ODR just means more samples per second to record.
static void setImu1kHz()
{
  M5.In_I2C.writeRegister8(0x68, 0x19, 0x00, 400000);  // SMPLRT_DIV = 0 -> 1 kHz
}

void setup() {

  M5.begin();

  setImu1kHz();

#if GYRO_FIFO_TEST
  // One-shot FIFO rate self-test: enable the gyro+accel FIFO and measure how
  // many packets/second the I2C bus can actually drain. Runs before the camera
  // connection so it doesn't interfere with the E2E test.
  // SMPLRT_DIV = 1 -> datasheet: 1kHz/(1+1) = 500 Hz on a spec chip. On this
  // clone the internal clock runs ~2.34x faster, so we expect ~1.17 kHz.
  DEBUG_INFO("[GYRO-FIFO] running FIFO rate self-test (SMPLRT_DIV=1)...");
  gyroLog.configureFifo(1); // SMPLRT_DIV = 1
  float fifoRate = gyroLog.measureFifoRate();
  DEBUG_INFO("[GYRO-FIFO] measured FIFO rate = %.1f Hz (target ~1.17 kHz)", fifoRate);
#endif

  tft.setColorDepth(16);
  tft.setSwapBytes(true);

  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.width());
  tft.setTextColor( TFT_WHITE);
  tft.setTextSize(1);
  tft.setFont(&Lato_Regular11pt7b);

  sprite = new LGFX_Sprite(&tft);
  sprite->setPsram(false);
  sprite->setColorDepth(BPP_SPRITE);
  sprite->setSwapBytes(true);
  sprite->setTextSize(1);
  sprite->setFont(&Lato_Regular11pt7b);


  /* To use Deep Sleep and use button B to wake up, the following code can be used.
      However, as the device won't maintain a connection while asleep, there isn't much point. Power button on the side can be used to shutdown.
      Power off: Quickly double-click the red power button on the left
      If you do want to go to deep sleep in your code, you can run the following code: esp_deep_sleep_start();

  pinMode(GPIO_NUM_38, INPUT_PULLUP); // Button B on M5Stack is GPIO 38, that's the one to wake up from sleep

  // Check if the ESP32 woke up from a deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.begin(115200);
    Serial.println("Woke up from sleep!");
    // Additional actions can be performed here after waking up
  } else {
    Serial.begin(115200);
    Serial.println("First boot!");
    // Additional setup can be performed here on the first boot
  }
  // Configure wake-up source
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, LOW);
  */

  // SET DEBUG LEVEL
  Debug.setDebugLevel(DBG_VERBOSE);
  Debug.timestampOn();

  // Allow a timeout of 30 seconds for time for the pass key entry. It's slower with buttons
  esp_task_wdt_init(35, true);

  // Splash screen
  tft.pushImage(0, 0, IWIDTH, IHEIGHT, MPCSplash_M5Stack_CoreS3);

  // Prepare for Bluetooth connections and start scanning for cameras
  cameraConnection.initialise(&tft, IWIDTH, IHEIGHT); // Screen Pass Key entry

  // When the connected camera's recording state changes, start/stop the gyro
  // log. The camera object is created when a connection is established, so we
  // register the handler lazily in loop() once a camera exists (see below).
}

int memoryLoopCounter;
bool forceRecordOutline = false; // Show the recording outline as we haven't done it yet

#if GYRO_E2E_TEST
// The E2E test's state machine. It runs on the main loop (called from loop())
// and drives the REAL camera record path, so it is a true end-to-end test:
//   0 = waiting to start (settle delay)
//   1 = recording: we sent the camera a real Record command and gyroLog.begin()
//       created the hello-world file. After GYRO_E2E_DURATION_S we send the
//       camera a real Preview (stop) command and set gyroPendingEnd (-> end()).
//   2 = waiting for the playback clip-name capture to rename the file to the
//       real clip name (the camera reports it after we flip it to playback).
//   3 = verify: read the (renamed) file back and check it exists + correct size.
//   4 = done (reported)
static int e2eState = 0;
static uint32_t e2eStartMs = 0;     // millis() when the record start was sent
static uint32_t e2eStopMs = 0;      // millis() when the record stop was sent
static uint32_t e2eConnectedMs = 0; // millis() when the camera first connected
static bool e2eConnectedSeen = false;
static std::string e2eClipName;      // the generic name we started the file with
static std::string e2eRealName;     // the real clip name the camera reported

static void gyroE2ETestTick()
{
  auto cam = BMDControlSystem::getInstance()->getCamera();
  bool connected = (cameraConnection.status == BMDCameraConnection::ConnectionStatus::Connected);

  // Note when the camera first connects, so we can settle for a fixed time
  // after connection (rather than a fixed time after boot).
  if(connected && !e2eConnectedSeen)
  {
    e2eConnectedSeen = true;
    e2eConnectedMs = millis();
  }

  switch(e2eState)
  {
    case 0:
    {
      // Wait until the camera is connected AND the post-connection settle delay
      // has elapsed, then send the camera a real RECORD command and queue the
      // gyro-log start (which creates the hello-world file).
      if(connected && cam && cam->hasTransportMode() &&
         (millis() - e2eConnectedMs >= GYRO_E2E_SETTLE_MS))
      {
        e2eClipName = nextGyroClipName();
        gyroPendingStart.clipName = e2eClipName;
        gyroPendingStart.ext = "braw";
        gyroPendingStart.timecode = "00:00:00:00";
        gyroPendingStart.valid = true;

        TransportInfo ti = cam->getTransportMode();
        ti.mode = CCUPacketTypes::MediaTransportMode::Record;
        DEBUG_INFO("[GYRO-E2E] sending camera RECORD command");
        PacketWriter::writeTransportInfo(ti, &cameraConnection);

        e2eStartMs = millis();
        e2eState = 1;
        DEBUG_INFO("[GYRO-E2E] queued record START for '%s'", e2eClipName.c_str());
      }
      break;
    }
    case 1:
    {
      // Recording. After the target duration, send the camera a real PREVIEW
      // (stop) command. We do NOT set gyroPendingEnd here: the camera's own
      // record-stop callback (fired by the PREVIEW command) sets it and starts
      // the playback clip-name capture, exactly as it would for a real user.
      // Driving the real callbacks keeps this a true end-to-end test.
      if(millis() - e2eStartMs >= (uint32_t)GYRO_E2E_DURATION_S * 1000UL)
      {
        if(connected && cam && cam->hasTransportMode())
        {
          TransportInfo ti = cam->getTransportMode();
          ti.mode = CCUPacketTypes::MediaTransportMode::Preview;
          DEBUG_INFO("[GYRO-E2E] sending camera PREVIEW (stop) command");
          PacketWriter::writeTransportInfo(ti, &cameraConnection);
        }
        else
        {
          DEBUG_ERROR("[GYRO-E2E] not connected / no transport mode; cannot send PREVIEW");
        }

        e2eStopMs = millis();
        e2eState = 2;
        DEBUG_INFO("[GYRO-E2E] sent STOP after %d s; waiting for camera stop callback", GYRO_E2E_DURATION_S);
      }
      break;
    }
    case 2:
    {
      // Wait for the camera's record-stop callback to have run end() (the file
      // is closed once gyroLog is no longer recording), then wait for the
      // playback clip-name capture to rename the file to the real clip name.
      if(!gyroLog.isRecording())
      {
        if(gyroPendingSlateNameValid)
        {
          // The real clip name arrived; the main loop will apply it (rename).
          e2eRealName = gyroPendingSlateName;
          e2eState = 3;
          DEBUG_INFO("[GYRO-E2E] got real clip name '%s'; verifying renamed file", e2eRealName.c_str());
        }
        else if(millis() - e2eStopMs > 10000)
        {
          // No clip name within 10s (e.g. camera didn't report one). Verify the
          // file under its original generic name instead.
          e2eRealName = e2eClipName;
          e2eState = 3;
          DEBUG_INFO("[GYRO-E2E] no clip name after 10s; verifying original name '%s'", e2eClipName.c_str());
        }
      }
      break;
    }
    case 3:
    {
      // Verify the file (renamed to the real clip name, or the original name if
      // no clip name arrived) exists, has the expected size, and is complete.
      char path[128];
      snprintf(path, sizeof(path), "/%s.txt", e2eRealName.c_str());

      const GyroLogWriter::Summary& s = gyroLog.getSummary();

      // Expected size = header + (rows * 19). In mock mode the row width is a
      // fixed 19 bytes, so this is exact. (In non-mock mode the file is the
      // "hello world" payload, so we fall back to the old size check.)
#if GYRO_MOCK_DATA
      uint64_t expectedSize = (uint64_t)s.mockHeaderBytes + (uint64_t)s.mockRows * 19;
      DEBUG_INFO("[GYRO-E2E] expected size: %lu-byte header + %lu rows * 19 = %lu bytes",
        (unsigned long)s.mockHeaderBytes, (unsigned long)s.mockRows, (unsigned long)expectedSize);
#else
      uint64_t expectedSize = (uint64_t)snprintf(nullptr, 0, "hello world\nclip: %s\n", e2eRealName.c_str());
#endif

      if(!gyroLog.fileExists(path))
      {
        DEBUG_ERROR("[GYRO-E2E] FAIL: '%s' was not found on the SD card", path);
      }
      else
      {
        uint64_t actual = gyroLog.fileSize(path);
        bool sizeOk = (actual == expectedSize);
        DEBUG_INFO("[GYRO-E2E] '%s' size=%lu bytes (expected %lu) -> %s",
          path, (unsigned long)actual, (unsigned long)expectedSize, sizeOk ? "PASS" : "FAIL");
        if(!sizeOk)
          DEBUG_ERROR("[GYRO-E2E] FAIL: size %lu != expected %lu", (unsigned long)actual, (unsigned long)expectedSize);

#if GYRO_MOCK_DATA
        // Also confirm the file was written *completely*: re-read it and check
        // the row count and that the last "t" index is rows-1 (no dropped rows).
        bool complete = gyroLog.verifyFileComplete(path, s.mockRows);
        if(!complete)
          DEBUG_ERROR("[GYRO-E2E] FAIL: file body is not complete/well-formed");
        else
          DEBUG_INFO("[GYRO-E2E] file body complete (all %lu rows present, ends with newline)",
            (unsigned long)s.mockRows);
#endif
      }
      e2eState = 4;
      break;
    }
    case 4:
    default:
      // Done. Do nothing further; the normal UI continues.
      break;
  }
}
#endif // GYRO_E2E_TEST

void loop() {

  static unsigned long lastConnectedTime = 0;
  const unsigned long reconnectInterval = 5000;  // 5 seconds (milliseconds)

  unsigned long currentTime = millis();

#if GYRO_E2E_TEST
  gyroE2ETestTick();
#endif

  // ---- Gyro log: apply queued SD work on the main loop thread ----
  // The record start/stop callbacks (BLE notify thread) only set flags; the
  // actual SD work (begin/end/applySlateName) runs here, on the same thread
  // that will later sample, so there's no cross-thread race on the card.

  // Start a new log (file created) if a record start was queued.
  if(gyroPendingStart.valid)
  {
    gyroPendingStart.valid = false;
    gyroStartArmed = false; // clear the de-dup flag now that the start is handled
    if(gyroLog.begin(gyroPendingStart.clipName, gyroPendingStart.ext, gyroPendingStart.timecode))
      DEBUG_INFO("[GYRO] started log '%s'", gyroPendingStart.clipName.c_str());
    else
      DEBUG_INFO("[GYRO] failed to start log '%s' (SD not ready?)", gyroPendingStart.clipName.c_str());
  }

  // While recording, append one sample's worth of data per ~1 ms tick. (With
  // GYRO_MOCK_DATA this is the mock GCSV row writer; later it becomes the real
  // 1 kHz IMU sampler.) Called every loop() iteration so the row rate tracks the
  // real sampler even though the loop cadence is irregular.
  gyroLog.poll();

  // Finalise a just-stopped log (close file + commit to card).
  if(gyroPendingEnd)
  {
    gyroPendingEnd = false;
    if(gyroLog.end())
      DEBUG_INFO("[GYRO] ended log '%s'", gyroLog.getSummary().fileName.c_str());
  }

  // Apply the real clip name (learned via playback) to the finalised log.
  if(gyroPendingSlateNameValid && !gyroLog.isRecording())
  {
    gyroPendingSlateNameValid = false;
    gyroLog.applySlateName(gyroPendingSlateName, gyroVideoExtension(BMDControlSystem::getInstance()->getCamera().get()));
    DEBUG_INFO("[GYRO] applied slate name '%s' to log", gyroPendingSlateName.c_str());
  }

  // Compute the SD free-space figure (one time, after a clip has stopped and the
  // camera is idle). freeClusterCount() walks the whole FAT (~tens of seconds),
  // so it must not run in the middle of the record stop/playback sequence; by now
  // the main loop is otherwise idle, so the one-time block is acceptable. The
  // result is cached in the volume, so the Gyro Log screen reads it cheaply.
  if(gyroFreeSpacePending && !gyroLog.isRecording())
  {
    gyroFreeSpacePending = false;
    gyroLog.refreshFreeSpace();
  }

  if ((cameraConnection.status == BMDCameraConnection::ConnectionStatus::Disconnected || cameraConnection.status == BMDCameraConnection::ConnectionStatus::FailedPassKey) && currentTime - lastConnectedTime >= reconnectInterval) {
    
    if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::Disconnected)
      DEBUG_VERBOSE("Disconnected for too long, trying to reconnect");
    else
      DEBUG_VERBOSE("Failed Pass Key, trying to reconnect");

    // Set the status to Scanning and then show the NoConnection screen to render the Scanning page before starting the scan (which blocks so it can't render the Scanning page before it finishes)
    cameraConnection.status = BMDCameraConnection::ConnectionStatus::Scanning;
    Screen_NoConnection();

    cameraConnection.scan();

    if(connectedScreenIndex != Screens::NoConnection) // Move to the No Connection screen if we're not on it
      Screen_NoConnection();
  }
  else if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::Connected)
  {
    auto camera = BMDControlSystem::getInstance()->getCamera();

    // Register the recording-state callback once we have a camera object.
    // (The camera is created when a connection is established, so this is
    // done here rather than in setup().)
    static bool gyroCallbackRegistered = false;
    if(!gyroCallbackRegistered)
    {
      gyroCallbackRegistered = true;
      camera->setOnRecordingStateChanged([](bool recording)
      {
        auto cam = BMDControlSystem::getInstance()->getCamera();

        // The slate name the camera currently reports. The camera sends
        // "Next Clip" as a placeholder (it has no real clip name yet), so we
        // treat that as "no name".
        std::string slate;
        if(cam->hasSlateName())
          slate = cam->getSlateName();
        bool slateIsPlaceholder = (slate == "Next Clip" || slate == "next clip" || slate.empty());

        DEBUG_INFO("== REC %s ==  slate='%s' (placeholder=%d)  timecode='%s'",
          recording ? "START" : "STOP",
          slate.c_str(),
          (int)slateIsPlaceholder,
          cam->getTimecodeString().c_str());

        // While we are in playback reading the clip name, the camera toggles
        // between record/preview/play as we drive it. Ignore all of that; the
        // real start/stop of the *next* clip will arrive once we've flipped
        // back to record.
        if(gyroInPlayback)
          return;

        if(recording)
        {
          // ---- RECORD START ----
          // The camera can push the record state more than once in a row (e.g. a
          // repeated "record" notify right after we command it). Ignore any
          // start that arrives while one is already armed/active, so we never
          // start a second, spurious log (which would fail on the busy SD and
          // corrupt the start/stop state machine). gyroStartArmed is set
          // synchronously here (on the notify thread), so it catches a 2nd start
          // that arrives before the main loop has processed the first.
          if(gyroStartArmed || gyroLog.isRecording() || gyroPendingStart.valid)
          {
            DEBUG_INFO("[GYRO] ignoring duplicate record START (already armed/recording/queued)");
            return;
          }
          gyroStartArmed = true;

          // Remember the codec so we can build the right file extension later.
          if(cam->hasCodec())
            gyroStartCodec = cam->getCodec().basicCodec;

          // Queue a new gyro log for the main loop to start. We do NOT call
          // begin() here: it does SD work and must not run on this BLE notify
          // thread. The main loop (same thread that will later sample) calls
          // begin() next. We rename the log to the real clip name once the
          // camera tells us (via playback) after the clip stops.
          gyroPendingStart.clipName = nextGyroClipName();
          gyroPendingStart.ext = gyroVideoExtension(cam.get());
          gyroPendingStart.timecode = cam->getTimecodeString();
          gyroPendingStart.valid = true;

          // Turn the display off while recording.
          tft.setBrightness(0);
        }
        else
        {
          // ---- RECORD STOP ----
          // Record the end timecode, then ask the main loop to finalise the log
          // (gyroPendingEnd). end() does a blocking SD unmount/remount and must
          // not run on this BLE notify thread.
          gyroLog.setTimecodeAtEnd(cam->getTimecodeString());
          gyroPendingEnd = true;

          // Flip the camera into playback (from the main loop) so it reports
          // the real clip name. Mark that we're in the playback-reading phase.
          gyroInPlayback = true;

          // Bring the display back on and force a refresh of the screen.
          tft.setBrightness(127);
          lastRefreshedScreen = 0;
        }
      });

      // Register a handler for the slate name the camera pushes. When we have
      // just switched the camera into playback to learn the clip name, this is
      // where the real file name (e.g. "A010_08260408_C022") arrives. We use it
      // to rename the GCSV, then tell the main loop to switch the camera back.
      camera->setOnSlateNameReceived([](const std::string& name)
      {
        // Only act while we are in playback reading the clip name, and only on
        // a real name (not the "Next Clip" placeholder the camera sends at
        // other times). The camera re-sends the name repeatedly while in
        // playback, so once we've used it we flip back to record and clear
        // gyroInPlayback; any further repeats are then ignored.
        if(!gyroInPlayback)
          return;
        if(name == "Next Clip" || name == "next clip" || name.empty())
          return;

        DEBUG_INFO("[GYRO] got clip name from playback: '%s'", name.c_str());

        // Queue the real clip name for the main loop to apply.
        gyroPendingSlateName = name;
        gyroPendingSlateNameValid = true;

        // We have the name; tell the main loop to flip back to record.
        gyroInPlayback = false;
      });
    }

    if(static_cast<byte>(connectedScreenIndex) >= 100)
    {
      // Check if the initial payload has been fully received and if it was after the camera's last modified time, update the camera's modified time
      if(cameraConnection.getInitialPayloadTime() != ULONG_MAX && cameraConnection.getInitialPayloadTime() > camera->getLastModified())
        camera->setLastModified();

      switch(connectedScreenIndex)
      {
        case Screens::Dashboard:
          Screen_Dashboard();
          break;
        case Screens::Recording:
          Screen_Recording();
          break;
        case Screens::ISO:
          Screen_ISO();
          break;
        case Screens::ShutterAngleSpeed:
          if(camera->shutterValueIsAngle)
            Screen_ShutterAngle();
          else
            Screen_ShutterSpeed();
          break;
        case Screens::WhiteBalanceTintWB:
        case Screens::WhiteBalanceTintT:
          Screen_WBTint(connectedScreenIndex == Screens::WhiteBalanceTintWB);
          break;
        case Screens::Codec:
          Screen_Codec();
          break;
        case Screens::Resolution:
          Screen_Resolution();
          break;
        case Screens::Framerate:
          Screen_Framerate();
          break;
        case Screens::Media:
          Screen_Media();
          break;
        case Screens::Lens:
          Screen_Lens();
          break;
        case Screens::GyroLog:
          Screen_GyroLog();
          break;
      }
    }
    else
      Screen_Dashboard(true); // Was on disconnected screen, now we're connected go to the dashboard

    lastConnectedTime = currentTime;
  }
  else if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::ScanningFound)
  {
    DEBUG_DEBUG("Cameras found!");

    cameraConnection.connect(cameraConnection.cameraAddresses[0]);

    if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::FailedPassKey)
      DEBUG_DEBUG("Loop - Failed Pass Key");

    // Clear the screen so we can show the default screen cleanly
    tft.fillScreen(TFT_BLACK);

    // Land on the Gyro Log screen (the new default page) when connected.
    connectedScreenIndex = Screens::GyroLog;
    lastRefreshedScreen = 0;

    lastConnectedTime = currentTime;
  }
  else if(cameraConnection.status == BMDCameraConnection::ConnectionStatus::ScanningNoneFound)
  {
    DEBUG_VERBOSE("Status Scanning NONE Found. Marking as Disconnected.");
    cameraConnection.status = BMDCameraConnection::Disconnected;
    lastConnectedTime = currentTime;

    Screen_NoConnection();
  }

  // Keep track of the memory use to check that there aren't memory leaks (or significant memory leaks)
  /*
  if(Debug.getDebugLevel() >= DBG_VERBOSE && memoryLoopCounter++ % 400 == 0)
  {
    DEBUG_VERBOSE("Heap Size Free: %d of %d", ESP.getFreeHeap(), ESP.getHeapSize());
  }
  */

  // Buttons
  btnAPressed = false;
  btnBPressed = false;

  M5.update();

  if(M5.BtnA.wasPressed() || M5.BtnB.wasPressed() || M5.BtnC.wasPressed())
  {
    // Left button function changes on context
    // Only handle dashboard here
    if(M5.BtnA.wasPressed())
    {
      DEBUG_DEBUG("Button A");

      switch(connectedScreenIndex)
      {
        case Screens::Dashboard:
        case Screens::Recording:
        case Screens::ISO:
        case Screens::ShutterAngleSpeed:
        case Screens::WhiteBalanceTintWB:
        case Screens::WhiteBalanceTintT:
        case Screens::Codec:
        case Screens::Resolution:
        case Screens::Framerate:
        case Screens::Media:
        case Screens::Lens:
        case Screens::GyroLog:
          // Indicate to the other screens the first button has been pressed
          btnAPressed = true;
          break;
      }
    }
    else if(M5.BtnB.wasPressed())
    {
      DEBUG_DEBUG("Button B");

      switch(connectedScreenIndex)
      {
        case Screens::Dashboard:
        case Screens::Recording:

          // RECORD START/STOP FOR DASHBOARD AND RECORDING SCREEN
          if(connectedScreenIndex == Screens::Dashboard || connectedScreenIndex == Screens::Recording)
          {
            // Do we have a camera instance created (happens when connected)
            if(BMDControlSystem::getInstance()->hasCamera())
            {
                // Get the camera instance so we can check the state of it
                auto camera = BMDControlSystem::getInstance()->getCamera();

                // Only hit record if we have the Transport Mode info (knowing if it's recording) and we're not already recording.
                if(camera->hasTransportMode())
                {
                    // Record button
                    DEBUG_VERBOSE("Record Start/Stop");

                    auto transportInfo = camera->getTransportMode();

                    if(!camera->isRecording)
                      transportInfo.mode = CCUPacketTypes::MediaTransportMode::Record;
                    else
                      transportInfo.mode = CCUPacketTypes::MediaTransportMode::Preview;

                    // Send the packet to the camera to start recording
                    PacketWriter::writeTransportInfo(transportInfo, &cameraConnection);
                }
            }
          }
          break;
        case Screens::ISO:
        case Screens::ShutterAngleSpeed:
        case Screens::WhiteBalanceTintWB:
        case Screens::WhiteBalanceTintT:
        case Screens::Codec:
        case Screens::Resolution:
        case Screens::Framerate:
        case Screens::Media:
        case Screens::Lens:
        case Screens::GyroLog:
          // Indicate to the other screens the second button has been pressed
          btnBPressed = true;
          break;
      }
    }
    else if(M5.BtnC.wasPressed())
    {
      DEBUG_DEBUG("Button C > NEXT SCREEN");

      switch(connectedScreenIndex)
      {
        case Screens::GyroLog:
          connectedScreenIndex = Screens::Dashboard;
          break;
        case Screens::Dashboard:
          connectedScreenIndex = Screens::Recording;
          break;
        case Screens::Recording:
          connectedScreenIndex = Screens::ISO;
          break;
        case Screens::ISO:
          connectedScreenIndex = Screens::ShutterAngleSpeed;
          break;
        case Screens::ShutterAngleSpeed:
          connectedScreenIndex = Screens::WhiteBalanceTintWB;
          break;
        case Screens::WhiteBalanceTintWB:
          connectedScreenIndex = Screens::WhiteBalanceTintT;
          break;
        case Screens::WhiteBalanceTintT:
          connectedScreenIndex = Screens::Codec;
          break;
        case Screens::Codec:
          connectedScreenIndex = Screens::Resolution;
          break;
        case Screens::Resolution:
          connectedScreenIndex = Screens::Framerate;
          break;
        case Screens::Framerate:
          connectedScreenIndex = Screens::Media;
          break;
        case Screens::Media:
          connectedScreenIndex = Screens::Lens;
          break;
        case Screens::Lens:
          connectedScreenIndex = Screens::GyroLog;
          break;
      }

      lastRefreshedScreen = 0; // Forces a refresh
    }
  }

  // ---- Clip-name capture via playback ----
  // Driven entirely by the single gyroInPlayback flag:
  //   * just after a record stop we set it and (below) flip the camera into
  //     playback so it reports the real clip name;
  //   * the slate callback clears it once the name arrives;
  //   * a 6s timeout clears it too, in case no name ever arrives.
  // Whichever clears it, we then flip the camera back to preview. All the BLE
  // writes happen here in the main loop, never the notify handler.
  static bool gyroPlaybackCommandSent = false;
  static unsigned long gyroPlaybackWaitStart = 0;

  if(gyroInPlayback &&
      cameraConnection.status == BMDCameraConnection::ConnectionStatus::Connected)
  {
    // 1) Send the playback command once, right after the record stop.
    if(!gyroPlaybackCommandSent)
    {
      gyroPlaybackCommandSent = true;
      gyroPlaybackWaitStart = currentTime;
      auto cam = BMDControlSystem::getInstance()->getCamera();
      if(cam && cam->hasTransportMode())
      {
        TransportInfo ti = cam->getTransportMode();
        ti.mode = CCUPacketTypes::MediaTransportMode::Play;
        DEBUG_INFO("[GYRO] switching camera to PLAYBACK to read clip name");
        PacketWriter::writeTransportInfo(ti, &cameraConnection);
      }
      else
      {
        DEBUG_ERROR("[GYRO] no transport mode; can't switch to playback");
        gyroInPlayback = false;
      }
    }

    // 2) If no clip name has arrived within 6s, give up and flip back.
    if(gyroInPlayback && currentTime - gyroPlaybackWaitStart > 6000)
    {
      DEBUG_INFO("[GYRO] no clip name after 6s in playback; switching back");
      gyroInPlayback = false;
    }
  }

  // 3) Whenever we leave the playback-reading phase (name received or timeout),
  //    flip the camera back to PREVIEW. We must NOT use Record here: on a BMD
  //    camera, setting the transport mode to Record immediately *starts* a new
  //    clip, so "switching back to Record" would fire off a fresh recording.
  //    Preview is the idle standby state the camera sits in between clips.
  if(!gyroInPlayback && gyroPlaybackCommandSent &&
      cameraConnection.status == BMDCameraConnection::ConnectionStatus::Connected)
  {
    gyroPlaybackCommandSent = false;
    auto cam = BMDControlSystem::getInstance()->getCamera();
    if(cam && cam->hasTransportMode())
    {
      TransportInfo ti = cam->getTransportMode();
      ti.mode = CCUPacketTypes::MediaTransportMode::Preview;
      DEBUG_INFO("[GYRO] switching camera back to Preview");
      PacketWriter::writeTransportInfo(ti, &cameraConnection);
    }
    // The clip is fully finalised and the camera is idle. Ask the main loop to
    // compute the (slow) SD free-space figure now, so the Gyro Log screen can
    // show a real number. Done on the main loop (not the BLE thread) to avoid
    // a cross-thread race on the SD card.
    gyroFreeSpacePending = true;
  }

  delay(5);
}