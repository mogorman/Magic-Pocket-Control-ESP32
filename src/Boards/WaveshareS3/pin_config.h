#ifndef WAVESHARE_S3_LCD_154_PIN_CONFIG_H
#define WAVESHARE_S3_LCD_154_PIN_CONFIG_H

// Pin map for the Waveshare ESP32-S3-Touch-LCD-1.54.
// Source: https://docs.waveshare.com/ESP32-S3-Touch-LCD-1.54
//   MCU      : ESP32-S3R8 (8MB OPI PSRAM, 16MB flash)
//   Display  : 1.54" ST7789 240x240, 4-wire SPI
//   Touch    : CST816T, I2C + RST + INT
//   IMU      : QMI8658, I2C
//   TF card  : SD_MMC (4-bit)
//   Buttons  : three user buttons (minus/plus/power), active-low, pulled up
//   No AXP2101 PMU on this board - the panel is always powered.

// ---- Shared I2C bus (touch, IMU, audio) ----
#define IIC_SDA   42
#define IIC_SCL   41

// ---- ST7789 display (240x240, 4-wire SPI) ----
#define LCD_DC    45
#define LCD_CS    21
#define LCD_SCLK  38
#define LCD_MOSI  39
#define LCD_MISO  -1
#define LCD_RESET 40
#define LCD_BL    46   // backlight

// ---- Touch (CST816T) ----
#define TP_RST    47
#define TP_INT    48

// ---- TF card (SD_MMC, 4-bit) ----
#define SD_CLK    16
#define SD_CMD    15
#define SD_D0     17
#define SD_D1     18
#define SD_D2     13
#define SD_D3     14

// ---- User buttons, active-low, pulled up ----
// Three physical buttons on the board:
//   minus (KEY0)  -> up / select (drives the per-screen A/B option handlers)
//   power (KEY5)  -> record start/stop on Dashboard/Recording; "down" elsewhere
//   plus  (KEY4)  -> next / previous screen navigation
#define KEY0      0
#define KEY4      4
#define KEY5      5

// ---- QMI8658 IMU interrupts (not wired on this board; poll instead) ----
#define QMI_INT1  -1
#define QMI_INT2  -1

// ---- Screen size ----
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 240

#endif // WAVESHARE_S3_LCD_154_PIN_CONFIG_H
