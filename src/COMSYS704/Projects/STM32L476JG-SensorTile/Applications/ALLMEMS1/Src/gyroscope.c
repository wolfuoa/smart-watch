#include "gyroscope.h"
#include "SensorTile.h"
#include "SensorTile_bus.h"
#include "spi.h"
#include "ALLMEMS1_config.h"

#define LSM_GYR_CS_LOW()					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#define LSM_GYR_CS_HIGH()					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

extern SPI_HandleTypeDef hbusspi2;

static int32_t BSP_LSM6DSM_WriteReg_Gyro(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM6DSM_ReadReg_Gyro(uint16_t Reg, uint8_t *pdata, uint16_t len);

void gyro_init()
{
    uint8_t entry;

	// ---------------- CTRL2_G (0x11) ----------------
	/** bio
	 *
	 * The configuration register is used to configure the
	 * angular rate sensor in the gyroscope
	 *
	 ** current config
	 *
	 * 0b00010000
	 * - Continuous
	 * - Normal (Not Low Power)
	 * - 12.5Hz
	 *
	 */

	entry = 0x10;
	BSP_LSM6DSM_WriteReg_Gyro(0x11U, &entry, 1);

	// ----------------------------------------------------

	// ---------------- CTRL7_G (0x16) ----------------
	/** bio
	 *
	 * The configuration register is used to configure filtering
	 * and rounding in the Gyroscope
	 *
	 ** current config
	 *
	 * 0b00000000
	 *
	 */

	entry = 0x00U;
	BSP_LSM6DSM_WriteReg_Gyro(0x16U, &entry, 1);

	// ----------------------------------------------------
}

void gyro_read(GyroscopeData * ctx)
{
	uint8_t out[6];

	int16_t outx;
	int16_t outy;
	int16_t outz;

	BSP_LSM6DSM_ReadReg_Gyro(0x22U, &out[0], 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x24U, &out[2], 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x23U, &out[1], 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x25U, &out[3], 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x26U, &out[4], 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x27U, &out[5], 1);

	outx = (out[1] << 8); outx |= out[0];
	outy = (out[3] << 8);
	outy |= out[2];
	outz = (out[5] << 8);
	outz |= out[4];

	int negative;

	negative = (out[1] >> 7);
	if(negative)
		outx = (outx | ~((1 << 15) -1));
	negative = (out[3] >> 7);
	if(negative)
		outy = (outy | ~((1 << 15) -1));
	negative = (out[4] >> 7);
	if(negative)
		outz = (outz | ~((1 << 15) -1));
}

static int32_t BSP_LSM6DSM_WriteReg_Gyro(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	uint8_t dataReg = (uint8_t)Reg;

	/* CS Enable */
	LSM_GYR_CS_LOW();

	if (BSP_SPI2_Send(&dataReg, 1) != 1)
	{
		ret = BSP_ERROR_UNKNOWN_FAILURE;
	}

	if (BSP_SPI2_Send(pdata, len) != len)
	{
		ret = BSP_ERROR_UNKNOWN_FAILURE;
	}

	/* CS Disable */
	LSM_GYR_CS_HIGH();

	return ret;
}

static int32_t BSP_LSM6DSM_ReadReg_Gyro(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	uint8_t dataReg = (uint8_t)Reg;

	/* CS Enable */
	LSM_GYR_CS_LOW();
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
	LSM_GYR_CS_HIGH();
	SPI_1LINE_TX(&hbusspi2);
	__HAL_SPI_ENABLE(&hbusspi2);
	return ret;
}
