// Waveshare ESP32-S3-Touch-LCD-1.54 display driver.
//
// Drives the onboard 1.54" ST7789 240x240 SPI LCD with the CST816T capacitive
// touch controller, using M5GFX (a LovyanGFX fork) in *standalone* mode - i.e.
// without M5Unified. This lets the rest of the app use the normal M5GFX
// drawing API (fillScreen / drawString / pushImage / getTouch ...) unchanged.
//
// Pin map (see pin_config.h):
//   ST7789 : SCLK=38 MOSI=39 MISO=-1 DC=45 CS=21 RST=40  backlight=46
//   Touch  : CST816T @0x15 on I2C (SDA=42 SCL=41), RST=47 INT=48
//
// The panel and the touch share the same I2C/SPI host, so the touch is attached
// to the panel (bus_shared) rather than as a separate device.

#ifndef WAVESHARE_S3_DISPLAY_H
#define WAVESHARE_S3_DISPLAY_H

#include "M5GFX.h"
#include "lgfx/v1/panel/Panel_ST7789.hpp"
#include "lgfx/v1/platforms/esp32/Bus_SPI.hpp"
#include "Boards/WaveshareS3/pin_config.h"

class WaveshareS3Display : public M5GFX
{
public:
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;

  WaveshareS3Display()
  {
    // SPI bus for the ST7789 (matches the working BambuHelper ws_lcd_154 config).
    {
      auto cfg = _bus.config();
      cfg.spi_host   = SPI2_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 80000000;
      cfg.freq_read  = 16000000;
      cfg.pin_sclk   = LCD_SCLK;
      cfg.pin_mosi   = LCD_MOSI;
      cfg.pin_miso   = LCD_MISO;
      cfg.pin_dc     = LCD_DC;
      cfg.use_lock   = true;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    // ST7789 panel: 240x240 visible, GRAM is 240x320 (top 240 rows used).
    // No touch is attached here: the app drives the CST816T with a standalone
    // I2C reader (CST9220 driver), and the M5GFX touch init can hang the core-1
    // watchdog if the IC doesn't answer. invert is left at its default (false),
    // matching the working reference.
    {
      auto cfg = _panel.config();
      cfg.pin_cs   = LCD_CS;
      cfg.pin_rst  = LCD_RESET;
      cfg.pin_busy = -1;
      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.panel_width   = 240;
      cfg.panel_height  = 240;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      cfg.readable      = false;
      // This panel's ST7789 needs the display-inverting bit set for correct
      // colors (without it the screen shows inverted colors).
      cfg.invert        = true;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }

  // Bring the panel up. Backlight is a plain GPIO (no PWM) on this board.
  //
  // Two things matter here:
  //  1. The ST7789 needs a short settle window after power-up before the init
  //     command sequence will succeed; calling init() immediately after boot
  //     makes the panel hang and trips the core-1 task watchdog (TG1WDT reset
  //     loop). 500ms matches the working BambuHelper reference for this board.
  //  2. We must call init(&_panel) - the explicit-panel overload - NOT the
  //     no-arg init(). The no-arg init() virtual-dispatches to M5GFX's own
  //     init_impl() override, which tries to *autodetect an M5Stack board*
  //     (M5Stack / M5StickC / ...) and fails for a non-M5 board. init(panel)
  //     instead calls the base LGFX_Device::init_impl directly, which just
  //     brings up the panel we configured above.
  bool begin()
  {
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    delay(500);
    return init(&_panel);
  }
};

#endif // WAVESHARE_S3_DISPLAY_H
