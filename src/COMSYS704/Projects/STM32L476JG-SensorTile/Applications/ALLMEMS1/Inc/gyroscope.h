/**
  ******************************************************************************
  * @file    gyroscope.h
  * @author  nwol626, lhos118
  * @version 1.0.0
  * @date    20-09-2024
  * @brief   Gyroscope initialization and utility functions
  ******************************************************************************
  */
  
#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <stdint.h>

// ---------------- Public Typedef ----------------

typedef struct GyroscopeData_T
{
    uint32_t alpha_acc;
    uint32_t beta_acc;
    uint32_t gamma_acc;
} GyroscopeData;

// ------------------------------------------------

// --------- Public Function Declarations ---------

void gyro_init();
void gyro_read(GyroscopeData * ctx);

// ------------------------------------------------

#endif // GYROSCOPE_H