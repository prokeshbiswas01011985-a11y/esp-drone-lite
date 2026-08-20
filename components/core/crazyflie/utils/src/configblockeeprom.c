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
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * configblock.c - Simple static implementation of the config block
 *
 * Modified for esp-drone-lite: eeprom driver removed (2026-08).
 * The config block is no longer persisted; default values are returned.
 * Based on configblockeeprom.c from espressif/esp-drone (GPL-3.0).
 */
#define DEBUG_MODULE "CFGBLK"

#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "debug_cf.h"
#include "configblock.h"

static bool isInit = false;

int configblockInit(void)
{
  if (isInit) {
    return 0;
  }

  // No persistent storage in esp-drone-lite: defaults are used as-is.
  DEBUG_PRINTD("config block stub: using defaults\n");

  isInit = true;

  return 0;
}

bool configblockTest(void)
{
  return true;
}

/* Static accessors - always return the compile-time defaults */
int configblockGetRadioChannel(void)
{
  return RADIO_CHANNEL;
}

int configblockGetRadioSpeed(void)
{
  return RADIO_DATARATE;
}

uint64_t configblockGetRadioAddress(void)
{
  return RADIO_ADDRESS;
}

float configblockGetCalibPitch(void)
{
  return 0;
}

float configblockGetCalibRoll(void)
{
  return 0;
}
