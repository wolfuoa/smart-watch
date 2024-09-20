#include "accelerometer.h"
#include "SensorTile.h"
#include "SensorTile_bus.h"
#include "spi.h"
#include "ALLMEMS1_config.h"

#define LSM_ACC_CS_LOW()					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
#define LSM_ACC_CS_HIGH()					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

extern SPI_HandleTypeDef hbusspi2;

static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);

void acc_init()
{
	uint8_t entry;

	// Disable I2C
	entry = 0x01;
	BSP_LSM303AGR_WriteReg_Acc(0x23, &entry, 1);
}

void acc_read(AccelerometerData * ctx)
{
}

/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata,
										  uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	uint8_t dataReg = (uint8_t)Reg;

	/* CS Enable */
	LSM_ACC_CS_LOW();

	if (BSP_SPI2_Send(&dataReg, 1) != 1)
	{
		ret = BSP_ERROR_UNKNOWN_FAILURE;
	}

	if (BSP_SPI2_Send(pdata, len) != len)
	{
		ret = BSP_ERROR_UNKNOWN_FAILURE;
	}

	/* CS Disable */
	LSM_ACC_CS_HIGH();

	return ret;
}

/**
 * @brief  Read register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be read
 * @param  pdata the pointer to the data to be read
 * @param  len the length of the data to be read
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata,
										 uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	uint8_t dataReg = (uint8_t)Reg;

	/* CS Enable */
	LSM_ACC_CS_LOW();
	if (len > 1)
	{
		LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80 | 0x40);
	}
	else
	{
		LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
	}
	__HAL_SPI_DISABLE(&hbusspi2);
	SPI_1LINE_RX(&hbusspi2);

	if (len > 1)
	{
		LSM303AGR_SPI_Read_nBytes(&hbusspi2, (pdata), len);
	}
	else
	{
		LSM303AGR_SPI_Read(&hbusspi2, (pdata));
	}

	/* CS Disable */
	LSM_ACC_CS_HIGH();
	SPI_1LINE_TX(&hbusspi2);
	__HAL_SPI_ENABLE(&hbusspi2);
	return ret;
}
