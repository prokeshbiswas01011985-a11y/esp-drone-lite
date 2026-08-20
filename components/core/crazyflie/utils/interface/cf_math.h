/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Utilities to simplify unit testing
 *
 */

#pragma once

// esp-drone-lite: dsp_lib (xtensa_math) removed.
// Only scalar helpers are kept; matrix ops were used exclusively by the
// removed Kalman estimator. Based on the original cf_math.h from
// espressif/esp-drone (GPL-3.0), modified 2026-08.

#include <math.h>
#include <stdint.h>

#include "cfassert.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

static inline float xtensa_sqrt(float in)
{
    ASSERT(in >= 0.0f);
    return sqrtf(in);
}

static inline float limPos(float in)
{
    if (in < 0.0f) {
        return 0.0f;
    }

    return in;
}

static inline float clip1(float a)
{
    if (a < -1.0f) {
        return -1.0f;
    }

    if (a > 1.0f) {
        return 1.0f;
    }

    return a;
}
