#include "ScreenSecurityHandler.h"
#include "Fonts/Lato_Regular11pt7b.h" // Standard font

// Take in all the pointers we need access to to render the screen and handle touch
ScreenSecurityHandler::ScreenSecurityHandler(BMDCameraConnection* bmdCameraConnectionPtr, lgfx::v1::ITouch* touchPtr, M5GFX* windowPtr, int screenWidth, int screenHeight)
{
  _bmdCameraConnectionPtr = bmdCameraConnectionPtr;
  _touchPtr = touchPtr;
  _windowPtr = windowPtr;
  // _spritePassKeyPtr = spritePassKeyPtr;
  _screenWidth = screenWidth;
  _screenHeight = screenHeight;
}

// The PIN pad layout is authored for a 320px-wide screen (the original M5Stack
// layout). On a narrower panel (e.g. the 240px Waveshare 1.54) the rightmost
// column of keys would fall off-screen, so we scale every x coordinate by
// (screenWidth / 320). Y is left unscaled - the layout's y range (0..220) already
// fits a 240px-tall panel. The same scale is applied in KeyPressed() so the
// hit-test matches the drawn buttons.
static inline int scaleX(int x, int screenWidth)
{
    return (x * screenWidth) / 320;
}

// Triggers when a Pass Key is required
uint32_t ScreenSecurityHandler::onPassKeyRequest()
{
    // Update the connection status to requesting PassKey
    _bmdCameraConnectionPtr->status = BMDCameraConnection::NeedPassKey;

    // Allow 15 seconds to enter the pass key.
    unsigned long startTime = millis();
    unsigned long currentTime = 0;

    const int W = _screenWidth; // native layout is 320 wide; scale to this

    // Draw the screen
    _windowPtr->fillScreen(TFT_BLACK);

    // Left side
    _windowPtr->fillRect(0, 0, 13, _screenHeight, TFT_ORANGE);
    _windowPtr->fillRect(13, 0, 2, _screenHeight, TFT_DARKGREY);

    _windowPtr->setTextColor(TFT_WHITE);
    _windowPtr->drawString("Code:", scaleX(24, W), 7, &Lato_Regular11pt7b);

    // Draw the 11 buttons, left to right, top to bottom (x scaled to panel width)
    _windowPtr->fillSmoothRoundRect(scaleX(20, W), 30, 70, 60, 5, TFT_YELLOW); // 7
    _windowPtr->fillSmoothRoundRect(scaleX(95, W), 30, 70, 60, 5, TFT_YELLOW); // 8
    _windowPtr->fillSmoothRoundRect(scaleX(170, W), 30, 70, 60, 5, TFT_YELLOW); // 9
    _windowPtr->fillSmoothRoundRect(scaleX(245, W), 30, 70, 60, 5, TFT_RED); // Back

    _windowPtr->setTextColor(TFT_BLACK);
    _windowPtr->drawString("7", scaleX(50, W), 53, &Lato_Regular11pt7b);
    _windowPtr->drawString("8", scaleX(125, W), 53, &Lato_Regular11pt7b);
    _windowPtr->drawString("9", scaleX(200, W), 53, &Lato_Regular11pt7b);
    _windowPtr->fillTriangle(scaleX(269, W), 60, scaleX(289, W), 45, scaleX(289, W), 73, TFT_BLACK);

    _windowPtr->fillSmoothRoundRect(scaleX(20, W), 95, 70, 60, 5, TFT_YELLOW); // 4
    _windowPtr->fillSmoothRoundRect(scaleX(95, W), 95, 70, 60, 5, TFT_YELLOW); // 5
    _windowPtr->fillSmoothRoundRect(scaleX(170, W), 95, 70, 60, 5, TFT_YELLOW); // 6
    _windowPtr->fillSmoothRoundRect(scaleX(245, W), 95, 70, 125, 5, TFT_YELLOW); // 0

    _windowPtr->drawString("4", scaleX(50, W), 118, &Lato_Regular11pt7b);
    _windowPtr->drawString("5", scaleX(125, W), 118, &Lato_Regular11pt7b);
    _windowPtr->drawString("6", scaleX(200, W), 118, &Lato_Regular11pt7b);
    _windowPtr->drawString("0", scaleX(275, W), 151, &Lato_Regular11pt7b);

    _windowPtr->fillSmoothRoundRect(scaleX(20, W), 160, 70, 60, 5, TFT_YELLOW); // 1
    _windowPtr->fillSmoothRoundRect(scaleX(95, W), 160, 70, 60, 5, TFT_YELLOW); // 2
    _windowPtr->fillSmoothRoundRect(scaleX(170, W), 160, 70, 60, 5, TFT_YELLOW); // 3

    _windowPtr->drawString("1", scaleX(50, W), 174, &Lato_Regular11pt7b);
    _windowPtr->drawString("2", scaleX(125, W), 174, &Lato_Regular11pt7b);
    _windowPtr->drawString("3", scaleX(200, W), 174, &Lato_Regular11pt7b);

    // Log the computed button positions so the layout can be verified against the
    // panel width (the rightmost key column must stay within screenWidth).
    DEBUG_INFO("[PINPAD] panel %dx%d; rightmost key col x=%d..%d, 9/6/3 col x=%d..%d",
      _windowPtr->width(), _windowPtr->height(),
      scaleX(245, W), scaleX(315, W), scaleX(170, W), scaleX(240, W));

    bool pinComplete = false;
    std::vector<int> pinCodeArray;

    _windowPtr->setTextColor(TFT_WHITE);

    unsigned long lastTapTime = 0; // Ensure we don't count a tap as a double entry

    do
    {
        // Allow up to 15 seconds to enter the pass key
        currentTime = millis();

        if(currentTime - startTime >= 15000)
        {
            // Took too long!
            DEBUG_VERBOSE("15 seconds to enter pass key, time expired.");
            pinComplete = true;
            break;
        }
        else
        {
            // Use the left bar to show time left
            int reduceBar = ((currentTime - startTime) / 1000) * (_screenHeight / 15);
            _windowPtr->fillRect(0, 0, 13, reduceBar, TFT_BLACK);
        }

        // Show the pin code entered
        _windowPtr->fillRect(scaleX(95, W), 0, 100, 21, TFT_BLACK);
        for(int count = 0; count < pinCodeArray.size(); count++)
        {
            _windowPtr->drawString(String(pinCodeArray[count]), scaleX(95, W) + (count * 15), 7, &Lato_Regular11pt7b);
        }

        // Wait for Touches
        lgfx::touch_point_t tp[3];
        int nums = getTouchRaw(tp, 3);
        if (nums) {
            for (int i = 0; i < nums; ++i)
            {
                int touchx = tp[i].x;
                int touchy = tp[i].y;

                int pressedKey = KeyPressed(touchx, touchy);

                if(pressedKey > -2 && (currentTime - lastTapTime > 400))
                {
                    if(pressedKey == -1)
                    {
                        // Backspace
                        pinCodeArray.pop_back();
                    }
                    else
                    {
                        pinCodeArray.push_back(pressedKey);
                    }

                    lastTapTime = currentTime;
                }

                break; // We'll only process one touch (if they have more than one finger on screen only the first will count)
            }
        }

        if(pinCodeArray.size() == 6)
            pinComplete = true;
        }

    while(!pinComplete); // && !(pinCode >= 100000 && pinCode < 999999));

   // Convert the vector array of 6 numbers into a number
    if(pinCodeArray.size() == 6)
    {
        int result = 0;
        for (int i = 0; i < 6; i++) {
            result = result * 10 + pinCodeArray[i];
        }

        return result; // Here's the pass key heading to authenticate.
    }
    else
        return 0; // Didn't enter a 6 digit pass key in time.
}

void ScreenSecurityHandler::onPassKeyNotify(uint32_t pass_key)
{
    DEBUG_VERBOSE("onPassKeyNotify");
    pass_key += 1;
}

bool ScreenSecurityHandler::onConfirmPIN(uint32_t pin)
{
    DEBUG_VERBOSE("onConfirmPIN");
    return true;
}

bool ScreenSecurityHandler::onSecurityRequest()
{
    DEBUG_VERBOSE("onSecurityRequest");
    return true;
}

void ScreenSecurityHandler::onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl)
{
    DEBUG_VERBOSE("ScreenSecurityHandler::OnAuthenticationComplete");

    // Update the connection status based on the outcome of the authentication
    if(auth_cmpl.success)
        _bmdCameraConnectionPtr->status = BMDCameraConnection::Connected;
    else
        _bmdCameraConnectionPtr->status = BMDCameraConnection::FailedPassKey;
}

// Returns the key pressed
// 0-9 for numbers
// -1 for backspace
// -2 for nothing
// The x bounds are the 320-wide native layout scaled to the panel width (see
// scaleX above) so the hit-test matches the drawn buttons on any panel size.
int ScreenSecurityHandler::KeyPressed(int x, int y)
{
    const int W = _screenWidth;

    if(x >= scaleX(20, W) && x <= scaleX(90, W) && y >= 30 && y <= 90)
        return 7;
    else if(x >= scaleX(95, W) && x <= scaleX(165, W) && y >= 30 && y <= 90)
        return 8;
    else if(x >= scaleX(170, W) && x <= scaleX(240, W) && y >= 30 && y <= 90)
        return 9;
    else if(x >= scaleX(245, W) && y >= 30 && y <= 90)
        return -1;

    else if(x >= scaleX(20, W) && x <= scaleX(90, W) && y >= 95 && y <= 155)
        return 4;
    else if(x >= scaleX(95, W) && x <= scaleX(165, W) && y >= 95 && y <= 155)
        return 5;
    else if(x >= scaleX(170, W) && x <= scaleX(240, W) && y >= 95 && y <= 155)
        return 6;
    else if(x >= scaleX(245, W) && y >= 95 && y <= 210)
        return 0;

    else if(x >= scaleX(20, W) && x <= scaleX(90, W) && y >= 160 && y <= 220)
        return 1;
    else if(x >= scaleX(95, W) && x <= scaleX(165, W) && y >= 160 && y <= 220)
        return 2;
    else if(x >= scaleX(170, W) && x <= scaleX(240, W) && y >= 160 && y <= 220)
        return 3;

    return -2;
}

// This function is taken from the LGFX Panel_Device.hpp/.cpp files and is used to get the touch directly rather than through the "M5.Display" which resulted in a nullptr for the _touch pointer.
uint_fast8_t ScreenSecurityHandler::getTouchRaw(lgfx::v1::touch_point_t* tp, uint_fast8_t count)
{
    if (_touchPtr == nullptr)
    {
        DEBUG_ERROR("Touch Pointer is null in security handler.");
        return 0;
    }

// bool need_transaction = (getStartCount() && _touchPtr->config().bus_shared);
// if (need_transaction) { endTransaction(); }

    count = _touchPtr->getTouchRaw(tp, count);

// if (need_transaction) { beginTransaction(); }

    return count;
}
