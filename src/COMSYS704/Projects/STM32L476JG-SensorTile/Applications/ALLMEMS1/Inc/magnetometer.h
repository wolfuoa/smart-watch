/**
  ******************************************************************************
  * @file    magnetometer.h
  * @author  nwol626, lhos118
  * @version 1.0.0
  * @date    20-09-2024
  * @brief   Magnetometer initialization and utility functions
  ******************************************************************************
  */

#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <stdint.h>

// ---------------- Public Typedef ----------------

typedef struct MagnetometerData_t
{
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
} MagnetometerData;

// ------------------------------------------------

// --------- Public Function Declarations ---------

void mag_init();
void mag_read(MagnetometerData *ctx);
double mag_angle(MagnetometerData *ctx);
void mag_calibrate(double angle);

// ------------------------------------------------

#endif // MAGNETOMETER_H
