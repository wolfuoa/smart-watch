/**
  ******************************************************************************
  * @file    spi.h
  * @author  nwol626, lhos118
  * @version 1.0.0
  * @date    20-09-2024
  * @brief   SPI initialization and utility functions
  ******************************************************************************
  */
#ifndef SPI_H
#define SPI_H

#include "SensorTile.h"
#include "SensorTile_bus.h"

void InitLSM();
uint8_t Sensor_IO_SPI_CS_Init_All(void);
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef *xSpiHandle, uint8_t *val, uint16_t nBytesToRead);
void LSM303AGR_SPI_Read(SPI_HandleTypeDef *xSpiHandle, uint8_t *val);
void LSM303AGR_SPI_Write(SPI_HandleTypeDef *xSpiHandle, uint8_t val);

#endif // SPI_H
