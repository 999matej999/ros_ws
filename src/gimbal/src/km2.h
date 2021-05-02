/*
 * This file is part of the KAMBot project.
 * 
 *  Copyright (C) 2016 Frantisek Burian <bufran _at_ seznam.cz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef KM2_H
#define KM2_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include <inttypes.h>
#include "i2c.h"

// default I2C address of module
#define ADDR_KM2_DEFAULT      0x70

// KM2 registers
#define KM2_SPEED             0x00   // RW 2x uint16_t
#define KM2_ODOMETRY          0x01   // R- 2x uint32_t
#define KM2_STATUS            0x01   // RW 1x uint16_t

#define KM2_CMDRESET          0xE0   // -W no data
#define KM2_CMDLOAD           0xE1   // -W no data
#define KM2_CMDSTORE          0xE2   // -W no data

#define KM2_CFGMAXSPD         0xF0    // RW 1x int16_t
#define KM2_CFGTIMEOUT        0xF1    // RW 1x uint16_t
#define KM2_CFGTIMEOUTPWOFF   0xF2    // RW 1x uint16_t
#define KM2_CFGADDR           0xFF    // RW 1x uint8_t (or 2x uint8_t)

// KM2 register values
  
// KM2_CFGADDR -----------------------------------------------------------------
  
#define KM2_CFGADDR_ADDR          (0x7F << 1)  
#define KM2_CFGADDR_ADDR_(v)      ((v) << 1)
#define KM2_CFGADDR_BCASTEN       (0x01)      // if set, device will respond on addr 0 too

// functions
  
/**
 * Drive the motors. Speeds are computed from this formula:
 * 
 * rot_per_sec = ((fosc / timer_max) * left / M) / ((360/step) * microsteps)
 * 
 * rot_per_sec = (left / timer_max) * (fosc / M) * (step / (360 * microsteps))
 * 
 * Where:
 *  fosc is frequency of internal oscillator [8000000 Hz +- 10%]
 *  timer_max is maximum timer value to overflow [0xFFF]
 *  left is register value [0 .. 8192]
 *  M is internal interpolation division constant [80]
 *  microsteps is number of microsteps per one motor step [32]
 *  step is motor step size in degrees [1.8]
 * 
 * Hence partial eqns:
 *  (fosc / timer_max) is base frequency of timer pulse generator [1953 Hz +- 10%]
 *  ((fosc / timer_max) * left / M) is the frequency generated on the STEP pin [24.4 Hz - 100 kHz]
 *  ((360 / step) * microsteps) is total count of microsteps per one motor revolution [6400]
 *  (fosc / M) is maximum frequency that can be achieved [100 kHz]
 * 
 *  the maximum theoretical rotational speed is therefore 15.625 rot/s yielding 
 *  from odometry the maximum theoretical forward speed of 3.2 m/s using standard wheels
 * 
 * 
 * @param bus The I2C bus file descriptor
 * @param chip_addr I2C address of the motor board
 * @param left desired speed of left motor
 * @param right desired speed of right motor
 * @return -1 if failure (errno set)
 * @return 0 if success
 */
static inline int km2_drive(int bus, int chip_addr, int16_t left, int16_t right)
{
  // C99 compound-literals
  int16_t data[] = {left, right};
  return i2c_write_leint16_array(bus, chip_addr, KM2_SPEED, data, 2);
}

/**
 * Read the differential odometry from the KM board
 * 
 * The differential odometry is count of pulses from previous call of this function
 * 
 * @param bus The I2C bus file descriptor
 * @param chip_addr I2C address of the motor board
 * @param odoL[OUT] count of left motor ticks from previous call
 * @param odoR[OUT] count of right motor ticks from previous call
 * @return -1 if failure (errno set)
 * @return 2 if success
 */
static inline int km2_odometry(int bus, int chip_addr, int32_t *odoL, int32_t *odoR)
{
  int32_t values[2];
  
  int err = i2c_read_leint32_array(bus, chip_addr, KM2_ODOMETRY, values, 2);
  if (err == 2) {
    *odoL = values[0];
    *odoR = values[1];
  }
  
  return err;
}

static inline int km2_cfg_set_address(int bus, int chip_addr, uint8_t new_addr)
{
  // enable broadcast by default
  return i2c_write_leuint08(bus, chip_addr, KM2_CFGADDR, KM2_CFGADDR_ADDR_(new_addr) | KM2_CFGADDR_BCASTEN);
}


#ifdef __cplusplus
}
#endif

#endif /* KM2_H */

