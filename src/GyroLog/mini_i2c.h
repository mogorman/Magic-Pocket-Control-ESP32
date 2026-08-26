// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Adapted from https://github.com/VladimirP1/esp-gyrologger (LGPL-2.1).
//
// A minimal, interrupt-driven I2C master driver for the ESP32. It programs the
// I2C hardware FIFO/command registers directly and raises a hardware interrupt
// on transfer-complete, so a read never blocks the calling context. This is what
// lets the gyro logger sample the MPU6886 at its true 1 kHz ODR without a
// polling loop.
//
// Differences from the upstream version:
//   * The I2C port, SDA/SCL pins and the TX/RX FIFO address are parameterised
//     (upstream hard-codes I2C port 0). On the M5Stack Core2 the IMU sits on
//     I2C_NUM_1 (SDA=G21, SCL=G22), so we use port 1.
//   * The FIFO address and the clock/reset bits are selected per port.

#pragma once

#include <esp_err.h>

#include <stdint.h>

// The implementation (mini_i2c.c) is plain C, so the prototypes must carry C
// linkage when this header is included from C++ (e.g. gyro_imu.cpp).
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    I2C_STATUS_IDLE,
    I2C_STATUS_TIMEOUT,
    I2C_STATUS_ARB_LOST,
    I2C_STATUS_NACK,
    I2C_STATUS_ACTIVE
} i2c_status_t;

// Configure the I2C peripheral. `i2c_num` is the peripheral (0 or 1),
// `sda_pin`/`scl_pin` the GPIOs, `freq` the SCL frequency in Hz.
esp_err_t mini_i2c_init(int i2c_num, int sda_pin, int scl_pin, int freq);

esp_err_t mini_i2c_read_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t* bytes, uint8_t n_bytes);
esp_err_t mini_i2c_write_reg_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t byte);
esp_err_t mini_i2c_write_reg2_sync(uint8_t dev_adr, uint8_t reg_adr, uint8_t byte1, uint8_t byte2);
esp_err_t mini_i2c_write_n_sync(uint8_t* data, int len);

// Start an asynchronous read of `n_bytes` from `reg_adr`. When the transfer
// completes, `callback` is invoked from the I2C interrupt handler (ISR
// context). Call mini_i2c_read_reg_get_result() from within (or after) the
// callback to fetch the received bytes.
esp_err_t mini_i2c_read_reg_callback(uint8_t dev_adr, uint8_t reg_adr, uint8_t n_bytes,
                                      void (*callback)(void* args), void* callback_args);
esp_err_t mini_i2c_read_reg_get_result(uint8_t* bytes, uint8_t n_bytes);

esp_err_t mini_i2c_hw_fsm_reset();
esp_err_t mini_i2c_set_timing(int freq);

i2c_status_t mini_i2c_get_status();

// Enable or disable the I2C peripheral's interrupt line. While disabled, the
// interrupt handler (and any transfer it re-arms) cannot run, so a blocking
// *_sync() call is guaranteed not to race an in-flight asynchronous transfer.
// Use this to quiesce the bus before a one-shot blocking read/write that must
// not be interleaved with the interrupt-driven sampling loop.
void mini_i2c_set_intr_enabled(bool enabled);

// Some IMUs need a longer gap after the STOP condition before the next
// transaction; this doubles the stop-hold timing.
esp_err_t mini_i2c_double_stop_timing();

// (Internal) Reconfigure the I2C GPIO pins. Exposed so the bus-clear routine can
// restore the pins after a manual bus recovery.
esp_err_t i2c_conf_pins(int i2c_num, int sda_io_num, int scl_io_num,
                         bool sda_pullup_en, bool scl_pullup_en);

#ifdef __cplusplus
}
#endif
