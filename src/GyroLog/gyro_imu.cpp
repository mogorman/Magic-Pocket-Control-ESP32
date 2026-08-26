// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Adapted from https://github.com/VladimirP1/esp-gyrologger (LGPL-2.1).
//
// Interrupt-driven MPU6886 sampling for the M5Stack Core2.
//
// The MPU6886's on-chip FIFO is used: the sensor buffers samples in hardware,
// and we drain it from the I2C hardware interrupt (mini_i2c.c) as soon as each
// transfer completes. This decouples sampling from the main loop, so we hold a
// steady 1 kHz (the MPU6886's maximum ODR) even while the UI/BLE are busy.
//
// MPU6886 FIFO sample layout (14 bytes):
//   [0..1] AX  [2..3] AY  [4..5] AZ  [6..7] TEMP  [8..9] GX  [10..11] GY  [12..13] GZ
//
// We configure the sensor for:
//   SMPLRT_DIV = 0   -> 1 kHz ODR
//   CONFIG     = 1   -> 44 Hz DLPF (the setting that lets SMPLRT_DIV work at 1 kHz)
//   GYRO_CONFIG  = +/-2000 dps
//   ACCEL_CONFIG = +/-8 g
//   FIFO_EN      = gyro + accel (14-byte samples)

#include "gyro_imu.h"
#include "mini_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string.h>

// MPU6886 I2C address (7-bit). The Core2's internal IMU sits at 0x68.
#define MPU6886_ADDR 0x68

// MPU6886 registers.
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_FIFO_EN       0x23
#define REG_INT_STATUS    0x3A
#define REG_USER_CONTROL  0x6A
#define REG_PWR_MGMT_1    0x6B
#define REG_FIFO_COUNT_H  0x72
#define REG_FIFO_RW       0x74
#define REG_WHO_AM_I      0x75

// FIFO enable bits.
#define FIFO_EN_GYRO  ((1 << 6) | (1 << 5) | (1 << 4))
#define FIFO_EN_ACCEL (1 << 3)

// User-control bits.
#define UC_FIFO_EN    (1 << 6)
#define UC_FIFO_RESET (1 << 2)

// Power-management bits.
#define PWR1_RESET  (1 << 7)
#define PWR1_SLEEP  (1 << 6)
#define PWR1_CLKSEL_ZGYRO (3)

// The FIFO sample is 14 bytes (accel + temp + gyro).
#define FIFO_SAMPLE_SIZE 14

// GYRO_CONFIG full-scale select: 3 = +/-2000 dps.
#define GYRO_FS_2000DPS (3)
// ACCEL_CONFIG full-scale select: 2 = +/-8 g.
#define ACCEL_FS_8G (2)

// Scales matching the above FSRs, used by the one-shot calibration read.
#define GYRO_DPS_PER_LSB (2000.0 / 32768.0)
#define ACCEL_G_PER_LSB  (8.0 / 32768.0)

static gyro_sample_cb_t s_cb = NULL;
static volatile bool s_running = false;

static uint64_t s_prev_time_us = 0;

// Forward declarations: the two ISR callbacks reference each other.
static void IRAM_ATTR fifo_bytes_cb(void* arg);
static void IRAM_ATTR data_cb(void* arg);

// ---- Interrupt-driven FIFO drain -------------------------------------------
//
// The chain mirrors esp-gyrologger:
//   1. We ask for a 2-byte read of the FIFO count register.
//   2. When that transfer completes (I2C interrupt), the ISR calls
//      fifo_bytes_cb, which reads the count.
//   3. If at least one full sample (14 bytes) is buffered, we ask for a 14-byte
//      read of the FIFO data register; on completion the ISR calls data_cb,
//      which parses the sample, timestamps it, delivers it to the callback, and
//      re-arms step 1.
//   4. If fewer than 14 bytes are buffered, we simply re-arm step 1 (the count
//      will grow on the next sensor ODR tick).

static void IRAM_ATTR data_cb(void* arg) {
    (void)arg;
    if (!s_running) {
        return;
    }

    uint8_t tmp[FIFO_SAMPLE_SIZE];
    if (mini_i2c_read_reg_get_result(tmp, FIFO_SAMPLE_SIZE) == ESP_OK) {
        uint64_t now_us = esp_timer_get_time();
        uint32_t dur_ns = (uint32_t)((now_us - s_prev_time_us) * 1000);
        s_prev_time_us = now_us;

        int16_t ax = (int16_t)((tmp[0] << 8) | tmp[1]);
        int16_t ay = (int16_t)((tmp[2] << 8) | tmp[3]);
        int16_t az = (int16_t)((tmp[4] << 8) | tmp[5]);
        int16_t gx = (int16_t)((tmp[8] << 8) | tmp[9]);
        int16_t gy = (int16_t)((tmp[10] << 8) | tmp[11]);
        int16_t gz = (int16_t)((tmp[12] << 8) | tmp[13]);

        if (s_cb) {
            s_cb(dur_ns, gx, gy, gz, ax, ay, az);
        }
    } else {
        // The read failed (NACK/timeout); reset the I2C FSM so we can recover.
        mini_i2c_hw_fsm_reset();
    }

    // Re-arm: check the FIFO count again.
    mini_i2c_read_reg_callback(MPU6886_ADDR, REG_FIFO_COUNT_H, 2, fifo_bytes_cb, NULL);
}

static void IRAM_ATTR fifo_bytes_cb(void* arg) {
    (void)arg;
    if (!s_running) {
        return;
    }

    uint8_t tmp[2];
    int fifo_bytes = 0;
    if (mini_i2c_read_reg_get_result(tmp, 2) == ESP_OK) {
        fifo_bytes = tmp[1] | (tmp[0] << 8);

        // The MPU6886 FIFO holds 1024 bytes = 73 samples. If we ever see more
        // than that, the drain has stalled; flag it (upstream hangs the CPU).
        if (fifo_bytes > 900) {
            // Best-effort: reset the FIFO so we can recover.
            mini_i2c_write_reg_sync(MPU6886_ADDR, REG_USER_CONTROL, UC_FIFO_RESET);
        }
    } else {
        mini_i2c_hw_fsm_reset();
    }

    if (fifo_bytes >= FIFO_SAMPLE_SIZE) {
        mini_i2c_read_reg_callback(MPU6886_ADDR, REG_FIFO_RW, FIFO_SAMPLE_SIZE, data_cb, NULL);
    } else {
        mini_i2c_read_reg_callback(MPU6886_ADDR, REG_FIFO_COUNT_H, 2, fifo_bytes_cb, NULL);
    }
}

bool gyro_imu_start(gyro_sample_cb_t cb) {
    s_cb = cb;

    // Configure the I2C bus: I2C_NUM_1, SDA=G21, SCL=G22, 400 kHz.
    if (mini_i2c_init(1, 21, 22, 400000) != ESP_OK) {
        return false;
    }

    // Verify the sensor is present.
    uint8_t who = 0;
    if (mini_i2c_read_reg_sync(MPU6886_ADDR, REG_WHO_AM_I, &who, 1) != ESP_OK) {
        return false;
    }
    // The MPU6886 reports 0x19; the MPU6050 reports 0x68. Both share the FIFO
    // layout we use here (we only support the 14-byte MPU6886 sample).
    if (who != 0x19 && who != 0x68) {
        return false;
    }

    // Reset the sensor, then wake it.
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_PWR_MGMT_1, PWR1_RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_PWR_MGMT_1, 0);

    // Use the internal gyro oscillator as the clock source.
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_PWR_MGMT_1, PWR1_CLKSEL_ZGYRO << 0);

    // 1 kHz ODR: SMPLRT_DIV = 0, DLPF = 44 Hz (CONFIG bit 0 set).
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_CONFIG, 1);
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_SMPLRT_DIV, 0);

    // Full-scale ranges.
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_GYRO_CONFIG, GYRO_FS_2000DPS << 3);
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_ACCEL_CONFIG, ACCEL_FS_8G << 3);

    // Enable the FIFO for gyro + accel (14-byte samples), and enable FIFO mode.
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_FIFO_EN, FIFO_EN_GYRO | FIFO_EN_ACCEL);
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_USER_CONTROL, UC_FIFO_EN);

    // Start the interrupt-driven drain loop.
    s_prev_time_us = esp_timer_get_time();
    s_running = true;
    mini_i2c_read_reg_callback(MPU6886_ADDR, REG_FIFO_COUNT_H, 2, fifo_bytes_cb, NULL);

    return true;
}

void gyro_imu_stop(void) {
    s_running = false;
    s_cb = NULL;
    // Reset the FIFO so any pending samples are discarded.
    mini_i2c_write_reg_sync(MPU6886_ADDR, REG_USER_CONTROL, UC_FIFO_RESET);
}

bool gyro_imu_read_now(float* gx, float* gy, float* gz, float* ax, float* ay, float* az) {
    // One-shot blocking read of the data registers (0x3B..0x48 = 14 bytes).
    uint8_t buf[14];
    if (mini_i2c_read_reg_sync(MPU6886_ADDR, 0x3B, buf, 14) != ESP_OK) {
        return false;
    }
    *ax = (int16_t)((buf[0] << 8) | buf[1]) * ACCEL_G_PER_LSB;
    *ay = (int16_t)((buf[2] << 8) | buf[3]) * ACCEL_G_PER_LSB;
    *az = (int16_t)((buf[4] << 8) | buf[5]) * ACCEL_G_PER_LSB;
    *gx = (int16_t)((buf[8] << 8) | buf[9]) * GYRO_DPS_PER_LSB * (3.14159265f / 180.0f);
    *gy = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_DPS_PER_LSB * (3.14159265f / 180.0f);
    *gz = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_DPS_PER_LSB * (3.14159265f / 180.0f);
    return true;
}
