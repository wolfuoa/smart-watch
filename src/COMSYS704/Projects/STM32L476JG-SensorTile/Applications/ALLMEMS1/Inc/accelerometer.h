/**
  ******************************************************************************
  * @file    accelerometer.h
  * @author  lhos118, nwol626
  * @version 1.0.0
  * @date    20-09-2024
  * @brief   Accelerometer initialization and utility functions
  ******************************************************************************
  */

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>

// ---------------- Public Typedef ----------------

typedef struct AccelerometerData_t
{
    int32_t x_acc;
    int32_t y_acc;
    int32_t z_acc;
} AccelerometerData;

// ------------------------------------------------

// --------- Public Function Declarations ---------

void acc_init();
void acc_read(AccelerometerData *ctx);

// ------------------------------------------------

#endif // ACCELEROMETER_H
