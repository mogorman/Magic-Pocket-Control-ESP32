#ifndef WAVESHARE_S3_AMOLED_216_PIN_CONFIG_H
#define WAVESHARE_S3_AMOLED_216_PIN_CONFIG_H

// Pin map for the Waveshare ESP32-S3-Touch-AMOLED-2.16.
// Source: https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-2.16
//   MCU      : ESP32-S3R8 (8MB PSRAM, 16MB flash)
//   Display  : 2.16" AMOLED 480x480, CO5300, QSPI
//   Touch    : CST9220 (CST816T protocol), I2C + INT + RST
//   IMU      : QMI8658, I2C
//   PMU      : AXP2101, I2C
//   TF card  : SD SPI
//   Button   : one user button (Key3) on GPIO18, pulled up

// ---- Shared I2C bus (touch, PMU, IMU, RTC, audio) ----
#define IIC_SDA   15
#define IIC_SCL   14

// ---- AMOLED display (CO5300, QSPI) ----
#define LCD_CS    12
#define LCD_SCLK  38
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_RESET 39

// ---- Touch (CST9220) ----
#define TP_RST    40
#define TP_INT    11

// ---- TF card (SD SPI) ----
#define SD_CS     41
#define SD_MOSI   1
#define SD_MISO   3
#define SD_SCK    2

// ---- User buttons, pulled up to VCC3V3 ----
// Three physical buttons on the board: A (GPIO0), B (GPIO16), C (GPIO18).
//   A (KEY0)  -> up / select (drives the per-screen A/B option handlers)
//   B (KEY16) -> record start/stop on Dashboard/Recording; "down" elsewhere
//   C (KEY18) -> next / previous screen navigation
#define KEY0      0
#define KEY16     16
#define KEY3      18

// ---- QMI8658 IMU interrupts ----
#define QMI_INT1  17
#define QMI_INT2  21

// ---- Screen size ----
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 480

#endif // WAVESHARE_S3_AMOLED_216_PIN_CONFIG_H
