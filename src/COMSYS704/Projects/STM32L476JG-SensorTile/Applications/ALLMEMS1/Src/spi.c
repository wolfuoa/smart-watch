#include "spi.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>

#define BSP_LSM6DSM_INT2_GPIO_PORT			 GPIOA
#define BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE()	 __GPIOA_CLK_ENABLE()
#define BSP_LSM6DSM_INT2_GPIO_CLK_DISABLE()	 __GPIOA_CLK_DISABLE()
#define BSP_LSM6DSM_INT2					 GPIO_PIN_2
#define BSP_LSM6DSM_INT2_EXTI_IRQn			 EXTI2_IRQn

#define BSP_LSM6DSM_CS_PORT					 GPIOB
#define BSP_LSM6DSM_CS_PIN					 GPIO_PIN_12
#define BSP_LSM6DSM_CS_GPIO_CLK_ENABLE()	 __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_M_CS_PORT				 GPIOB
#define BSP_LSM303AGR_M_CS_PIN				 GPIO_PIN_1
#define BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_X_CS_PORT				 GPIOC
#define BSP_LSM303AGR_X_CS_PIN				 GPIO_PIN_4
#define BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE() __GPIOC_CLK_ENABLE()

#define BSP_LPS22HB_CS_PORT					 GPIOA
#define BSP_LPS22HB_CS_PIN					 GPIO_PIN_3
#define BSP_LPS22HB_CS_GPIO_CLK_ENABLE()	 __GPIOA_CLK_ENABLE()

/**
 * @brief  Set all sensor Chip Select high. To be called before any SPI
 * read/write
 * @param  None
 * @retval HAL_StatusTypeDef HAL Status
 */
uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	uint8_t inData[2];
	/* Set all the pins before init to avoid glitch */
	BSP_LSM6DSM_CS_GPIO_CLK_ENABLE();
	BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE();
	BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE();
	BSP_LPS22HB_CS_GPIO_CLK_ENABLE();

	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,
					  GPIO_PIN_SET);
	HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,
					  GPIO_PIN_SET);
	HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStruct.Pin = BSP_LSM6DSM_CS_PIN;
	HAL_GPIO_Init(BSP_LSM6DSM_CS_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = BSP_LSM303AGR_X_CS_PIN;
	HAL_GPIO_Init(BSP_LSM303AGR_X_CS_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,
					  GPIO_PIN_SET);

	GPIO_InitStruct.Pin = BSP_LSM303AGR_M_CS_PIN;
	HAL_GPIO_Init(BSP_LSM303AGR_M_CS_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,
					  GPIO_PIN_SET);

	GPIO_InitStruct.Pin = BSP_LPS22HB_CS_PIN;
	HAL_GPIO_Init(BSP_LPS22HB_CS_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

	// setup SPI interface
	if (BSP_SPI2_Init() == BSP_ERROR_NONE)
	{
		// XPRINTF("NO SPI Error\r\n");
	}

	// disable I2C modes on all SPI devices
	// LSM303 Mag
	HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,
					  GPIO_PIN_RESET);
	inData[0] = (0x62U);
	BSP_SPI2_Send(inData, 1);
	inData[0] = 0x20;
	BSP_SPI2_Send(inData, 1);
	HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,
					  GPIO_PIN_SET);

	// LSM303 Acc
	HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,
					  GPIO_PIN_RESET);
	inData[0] = (0x23U);
	BSP_SPI2_Send(inData, 1);
	inData[0] = 0x01;
	BSP_SPI2_Send(inData, 1);
	HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,
					  GPIO_PIN_SET);

	// LPS22HB
	HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_RESET);
	inData[0] = (0x10U);
	BSP_SPI2_Send(inData, 1);
	inData[0] = 0x01;
	BSP_SPI2_Send(inData, 1);
	HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

	// LSM6DSM
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);
	inData[0] = (0x12U);
	BSP_SPI2_Send(inData, 1);
	inData[0] = 0x0C;
	BSP_SPI2_Send(inData, 1);
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

	return HAL_OK;
}

/**
 * @brief  This function reads multiple bytes on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @param  nBytesToRead: number of bytes to read.
 * @retval None
 */
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef *xSpiHandle, uint8_t *val,
							   uint16_t nBytesToRead)
{
	/* Interrupts should be disabled during this operation */
	__disable_irq();
	__HAL_SPI_ENABLE(xSpiHandle);

	/* Transfer loop */
	while (nBytesToRead > 1U)
	{
		/* Check the RXNE flag */
		if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
		{
			/* read the received data */
			*val = *(__IO uint8_t *)&xSpiHandle->Instance->DR;
			val += sizeof(uint8_t);
			nBytesToRead--;
		}
	}
	/* In master RX mode the clock is automaticaly generated on the SPI enable.
	So to guarantee the clock generation for only one data, the clock must be
	disabled after the first bit and before the latest bit of the last Byte
	received */
	/* __DSB instruction are inserted to garantee that clock is Disabled in the
	 * right timeframe */

	__DSB();
	__DSB();
	__HAL_SPI_DISABLE(xSpiHandle);

	__enable_irq();

	while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
	/* read the received data */
	*val = *(__IO uint8_t *)&xSpiHandle->Instance->DR;
	while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  This function send a command through SPI bus.
 * @param  command: command id.
 * @param  uint8_t val: value.
 * @retval None
 */
void LSM303AGR_SPI_Read(SPI_HandleTypeDef *xSpiHandle, uint8_t *val)
{
	/* In master RX mode the clock is automaticaly generated on the SPI enable.
	So to guarantee the clock generation for only one data, the clock must be
	disabled after the first bit and before the latest bit */
	/* Interrupts should be disabled during this operation */

	__disable_irq();
	__HAL_SPI_ENABLE(xSpiHandle);
	__asm("dsb\n");
	__asm("dsb\n");
	__HAL_SPI_DISABLE(xSpiHandle);
	__enable_irq();

	while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
	/* read the received data */
	//  XPRINTF("Before READ, %d,%d\r\n",val[0],xSpiHandle->Instance);
	*val = *(__IO uint8_t *)&xSpiHandle->Instance->DR;
	//  XPRINTF("READ, %d\r\n",val[0]);
	while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  This function send a command through SPI bus.
 * @param  command : command id.
 * @param  val : value.
 * @retval None
 */
void LSM303AGR_SPI_Write(SPI_HandleTypeDef *xSpiHandle, uint8_t val)
{
	/* check TXE flag */
	while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

	/* Write the data */
	*((__IO uint8_t *)&xSpiHandle->Instance->DR) = val;

	/* Wait BSY flag */
	while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
	while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}
