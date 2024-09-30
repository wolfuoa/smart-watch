#include "accelerometer.h"
#include "SensorTile.h"
#include "SensorTile_bus.h"
#include "spi.h"
#include "ALLMEMS1_config.h"
#include <math.h>

#define LSM_ACC_CS_LOW()					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
#define LSM_ACC_CS_HIGH()					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

// ---------------- Private Variables ----------------
int32_t xAccAvg = 0;
int32_t yAccAvg = 0;
int32_t zAccAvg = 0;
int32_t maxAccZ = 0;
// ---------------------------------------------------

extern SPI_HandleTypeDef hbusspi2;

static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);

void acc_init()
{
	uint8_t entry;

	// Disable I2C, enable SPI, 12-bit mode, xyz registers
	entry = 0x09;
	BSP_LSM303AGR_WriteReg_Acc(0x23, &entry, 1);
	entry = 0x57;
	BSP_LSM303AGR_WriteReg_Acc(0x20, &entry, 1);

	// Configure high pass filter 10001000
	entry = 0x88;
	BSP_LSM303AGR_WriteReg_Acc(0x21, &entry, 1);

}

void acc_read(AccelerometerData * ctx)
{
	uint8_t OUTX_L_A;
	uint8_t OUTX_H_A;
	uint8_t OUTY_L_A;
	uint8_t OUTY_H_A;
	uint8_t OUTZ_L_A;
	uint8_t OUTZ_H_A;

	int16_t outx;
	int16_t outy;
	int16_t outz;


	BSP_LSM303AGR_ReadReg_Acc(0x28, &OUTX_L_A, 1);
	BSP_LSM303AGR_ReadReg_Acc(0x29, &OUTX_H_A, 1);
	BSP_LSM303AGR_ReadReg_Acc(0x2A, &OUTY_L_A, 1);
	BSP_LSM303AGR_ReadReg_Acc(0x2B, &OUTY_H_A, 1);
	BSP_LSM303AGR_ReadReg_Acc(0x2C, &OUTZ_L_A, 1);
	BSP_LSM303AGR_ReadReg_Acc(0x2D, &OUTZ_H_A, 1);


	outx = (OUTX_H_A << 8);
	outy = (OUTY_H_A << 8);
	outz = (OUTZ_H_A << 8);

	outx |= OUTX_L_A;
	outy |= OUTY_L_A;
	outz |= OUTZ_L_A;

	int negative;

	negative = (OUTX_H_A >> 7);
	if(negative)
	{
		ctx->x_acc = outx | ~((1 << 15) -1);
	}
	else
	{
		ctx->x_acc = outx;
	}

	negative = (OUTY_H_A >> 7);
	if(negative)
	{
		ctx->y_acc = outy | ~((1 << 15) -1);
	}
	else
	{
		ctx->y_acc = outy;
	}

	negative = (OUTZ_H_A >> 7);
	if(negative)
	{
		ctx->z_acc = outz | ~((1 << 15) -1);
	}
	else
	{
		ctx->z_acc = outz;
	}



	XPRINTF("A=%d\t%d\t%d\t",ctx->x_acc,ctx->y_acc,ctx->z_acc);
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
