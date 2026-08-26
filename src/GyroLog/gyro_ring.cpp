// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Adapted from https://github.com/VladimirP1/esp-gyrologger (LGPL-2.1).
//
// Lock-free ring buffer for IMU samples. The producer (I2C interrupt) writes a
// slot and publishes the write pointer under a critical section; the consumer
// (GCSV writer task) reads slots until it catches up. The capacity is a power
// of two so the index wrap-around is a cheap mask.

#include "gyro_ring.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdlib.h>
#include <string.h>

static gyro_ring_sample_t* s_ring = NULL;
static int s_capacity = 0;
static int s_mask = 0;

// The write pointer is updated in ISR context and read in task context, so it
// is guarded by a critical section (a single int write/read is atomic on the
// ESP32, but the critical section makes the intent explicit and matches
// esp-gyrologger's approach).
static volatile int s_wptr = 0;
static int s_rptr = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Round `n` up to the next power of two (minimum 2).
static int next_pow2(int n) {
    int p = 2;
    while (p < n) {
        p <<= 1;
    }
    return p;
}

bool gyro_ring_init(int capacity) {
    if (s_ring) {
        return true; // already initialised
    }
    int cap = next_pow2(capacity < 2 ? 2 : capacity);
    s_ring = (gyro_ring_sample_t*)malloc(sizeof(gyro_ring_sample_t) * cap);
    if (!s_ring) {
        return false;
    }
    memset(s_ring, 0, sizeof(gyro_ring_sample_t) * cap);
    s_capacity = cap;
    s_mask = cap - 1;
    s_wptr = 0;
    s_rptr = 0;
    return true;
}

void gyro_ring_push(uint32_t duration_ns,
                     int16_t gx, int16_t gy, int16_t gz,
                     int16_t ax, int16_t ay, int16_t az) {
    if (!s_ring) {
        return;
    }

    gyro_ring_sample_t* slot = &s_ring[s_wptr & s_mask];
    slot->duration_ns = duration_ns;
    slot->gx = gx;
    slot->gy = gy;
    slot->gz = gz;
    slot->ax = ax;
    slot->ay = ay;
    slot->az = az;

    // Publish the new write pointer atomically with respect to the consumer.
    // This runs in ISR context, so use the ISR critical-section macros.
    int shadow = s_wptr + 1;
    portENTER_CRITICAL_ISR(&s_mux);
    s_wptr = shadow;
    portEXIT_CRITICAL_ISR(&s_mux);
}

bool gyro_ring_pop(gyro_ring_sample_t* out) {
    if (!s_ring) {
        return false;
    }

    // Read the current write pointer under the critical section.
    int wptr;
    portENTER_CRITICAL(&s_mux);
    wptr = s_wptr;
    portEXIT_CRITICAL(&s_mux);

    if (s_rptr == wptr) {
        return false; // empty
    }

    *out = s_ring[s_rptr & s_mask];
    s_rptr = (s_rptr + 1) & s_mask;
    return true;
}

int gyro_ring_count(void) {
    if (!s_ring) {
        return 0;
    }
    int wptr;
    portENTER_CRITICAL(&s_mux);
    wptr = s_wptr;
    portEXIT_CRITICAL(&s_mux);
    int n = (wptr - s_rptr) & s_mask;
    return n;
}
