// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Adapted from https://github.com/VladimirP1/esp-gyrologger (LGPL-2.1).
//
// Interrupt-driven MPU6886 sampling. The MPU6886's on-chip FIFO is drained by
// the I2C hardware interrupt (see mini_i2c.c), so sampling runs at the sensor's
// true 1 kHz ODR independent of the main loop. Each sample is delivered to a
// callback (the ring buffer) with a measured inter-sample interval.

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Called (from the I2C interrupt context) for every new sample.
//   duration_ns : measured time since the previous sample (nanoseconds)
//   gx..gz      : raw 16-bit gyro counts (already scaled to the configured FSR)
//   ax..az      : raw 16-bit accel counts
typedef void (*gyro_sample_cb_t)(uint32_t duration_ns,
                                  int16_t gx, int16_t gy, int16_t gz,
                                  int16_t ax, int16_t ay, int16_t az);

// Initialise the I2C bus (I2C_NUM_1, SDA=G21, SCL=G22 on the Core2), configure
// the MPU6886 for 1 kHz FIFO streaming, and start the interrupt-driven drain
// loop. `cb` is invoked for each sample. Returns true on success.
bool gyro_imu_start(gyro_sample_cb_t cb);

// Stop sampling and disable the FIFO. The I2C bus is left configured.
void gyro_imu_stop(void);

// One-shot read of the current gyro (rad/s) and accel (g) via a blocking I2C
// read of the data registers. Used by the calibration screen. Returns true on
// success.
bool gyro_imu_read_now(float* gx, float* gy, float* gz, float* ax, float* ay, float* az);

// Diagnostic: read back the ODR-relevant registers (SMPLRT_DIV, CONFIG/DLPF,
// PWR_MGMT_1, FIFO_EN) and measure the *actual* sample rate by counting how
// many samples the FIFO accumulates over a one-second window. Prints the
// results to serial. Returns the measured ODR in Hz (0 on failure). Call once
// at startup, after gyro_imu_start(), to confirm the sensor is really running
// at 1 kHz.
uint32_t gyro_imu_measure_odr(void);
