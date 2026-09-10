/*
    MIT License

  Copyright (c) 2021 Felix Biego

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef CST9220_H
#define CST9220_H

#include <Arduino.h>

// The Waveshare ESP32-S3-Touch-AMOLED-2.16 uses a CST9220 capacitive touch
// controller. It speaks the same I2C register protocol as the CST816T (the
// "CST816T" mode of the CST816S family): gesture id at 0x01, event + 12-bit
// x/y at 0x03. Its 7-bit I2C address is 0x15.
#define CST9220_ADDRESS     0x15

class CST9220 {

  public:

    // Gesture values reported in data.gestureID (CST816T protocol).
    enum GESTURE {
      NONE = 0x00,
      SWIPE_DOWN = 0x01,
      SWIPE_UP = 0x02,
      SWIPE_LEFT = 0x03,
      SWIPE_RIGHT = 0x04,
      SINGLE_CLICK = 0x05,
      DOUBLE_CLICK = 0x0B,
      LONG_PRESS = 0x0C
    };

    // The type of touch event reported in data.eventID.
    enum TOUCHEVENT {
      DOWN = 0x00,
      UP = 0x01,
      CONTACT = 0x02
    };

    struct data_struct {
      byte gestureID; // Gesture ID
      byte points;  // Number of touch points [NOT USED]
      byte eventID; // Event (0 = Down, 1 = Up, 2 = Contact)
      int x;
      int y;
      uint8_t version; // [NOT USED]
      uint8_t versionInfo[3]; // [NOT USED]
    };

    CST9220(int sda, int scl, int rst, int irq);
    void begin(int interrupt = RISING);
    void sleep();
    bool available();
    data_struct data;
    String gesture();
    String event();

  private:
    int _sda;
    int _scl;
    int _rst;
    int _irq;
    bool _event_available;

    void IRAM_ATTR handleISR();
    void read_touch();

    uint8_t i2c_read(uint16_t addr, uint8_t reg_addr, uint8_t * reg_data, uint32_t length);
    uint8_t i2c_write(uint8_t addr, uint8_t reg_addr, const uint8_t * reg_data, uint32_t length);

    bool getTouchCST816T(uint16_t *x, uint16_t *y, uint8_t *gesture, uint8_t *event);
    uint8_t i2c_read(uint8_t addr);
    uint8_t i2c_read_continuous(uint8_t addr, uint8_t *data, uint32_t length);
};

#endif
