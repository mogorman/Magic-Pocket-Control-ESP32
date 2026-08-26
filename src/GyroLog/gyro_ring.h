// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Adapted from https://github.com/VladimirP1/esp-gyrologger (LGPL-2.1).
//
// A small lock-free ring buffer that decouples the I2C-interrupt-driven IMU
// sampler (producer, ISR context) from the GCSV writer task (consumer). The
// producer only ever writes one slot and publishes a write pointer under a
// critical section; the consumer reads slots until it catches up. If the
// consumer falls behind, the oldest samples are dropped (the write pointer
// laps the read pointer), which is the correct behaviour for a real-time log.

#pragma once

#include <stdint.h>
#include <stdbool.h>

// One IMU sample: a measured inter-sample interval plus the raw 16-bit counts.
typedef struct {
    uint32_t duration_ns;
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
} gyro_ring_sample_t;

// Create a ring of `capacity` samples. Returns true on success.
bool gyro_ring_init(int capacity);

// Producer (ISR context): store a sample and advance the write pointer.
void gyro_ring_push(uint32_t duration_ns,
                    int16_t gx, int16_t gy, int16_t gz,
                    int16_t ax, int16_t ay, int16_t az);

// Consumer (task context): pop the next sample. Returns true if a sample was
// delivered, false if the ring is currently empty.
bool gyro_ring_pop(gyro_ring_sample_t* out);

// Number of samples currently buffered (approximate, for diagnostics).
int gyro_ring_count(void);
