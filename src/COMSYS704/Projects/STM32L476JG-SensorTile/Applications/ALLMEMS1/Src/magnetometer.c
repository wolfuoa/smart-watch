#include "magnetometer.h"
#include "SensorTile.h"
#include "SensorTile_bus.h"
#include "spi.h"
#include "ALLMEMS1_config.h"
#include <math.h>

#define LSM_MAG_CS_LOW()					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
#define LSM_MAG_CS_HIGH()					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
#define PI 3.1415926F

double calibration_angle = 0.0;

extern SPI_HandleTypeDef hbusspi2;

static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata,  uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len);

void mag_init()
{
	uint8_t entry;

	// Disable I2C
	entry = 0x20;
	BSP_LSM303AGR_WriteReg_Mag(0x62U, &entry, 1);

	// ---------------- CFG_REG_A_M (0x60) ----------------
	/** bio
	 *
	 * The configuration register is used to configure the output
	 * data rate and the measurement configuration.
	 *
	 ** current config
	 *
	 * 0b00000000
	 * - Continuous
	 * - Normal (Not Low Power)
	 * - 10Hz
	 *
	 */

	entry = 0x00U;
	BSP_LSM303AGR_WriteReg_Mag(0x60U, &entry, 1);

	// ----------------------------------------------------

	// ---------------- CFG_REG_B_M (0x61) ----------------
	/** bio
	 *
	 * The configuration register is used to configure offset
	 * calculation, set pulse frequency, and low-pass digital
	 * filtering
	 *
	 ** current config
	 *
	 * 0b00000010
	 *
	 */

	entry = 0x02U;
	BSP_LSM303AGR_WriteReg_Mag(0x61U, &entry, 1);

	// ----------------------------------------------------

	// ---------------- CFG_REG_C_M (0x62) ----------------
	/** bio
	 *
	 * The configuration register is used to configure I2C
	 * interface, data inversion, and digital output
	 *
	 ** current config
	 *
	 * 0b00000001
	 * - Use DRDY instead of guessing
	 *
	 */

	entry = 0x01U;
	BSP_LSM303AGR_WriteReg_Mag(0x62U, &entry, 1);

	// ----------------------------------------------------
}

void mag_read(MagnetometerData * ctx)
{
	uint8_t outx_l;
	uint8_t outx_h;
	uint8_t outy_l;
	uint8_t outy_h;
	uint8_t outz_l;
	uint8_t outz_h;

	int16_t outx;
	int16_t outy;
	int16_t outz;

	BSP_LSM303AGR_ReadReg_Mag(0x68U, &outx_l, 1);
	BSP_LSM303AGR_ReadReg_Mag(0x69U, &outx_h, 1);
	BSP_LSM303AGR_ReadReg_Mag(0x6AU, &outy_l, 1);
	BSP_LSM303AGR_ReadReg_Mag(0x6BU, &outy_h, 1);
	BSP_LSM303AGR_ReadReg_Mag(0x6CU, &outz_l, 1);
	BSP_LSM303AGR_ReadReg_Mag(0x6DU, &outz_h, 1);

	outx = (outx_h << 8);
	outx |= outx_l;

	outy = (outy_h << 8);
	outy |= outy_l;

	outz = (outz_h << 8);
	outz |= outz_l;

	int negative;

	negative = (outx_h >> 7);
	if(negative)
		outx = (outx | ~((1 << 15) -1));

	negative = (outy_h >> 7);
	if(negative)
		outy = (outy | ~((1 << 15) -1));

	negative = (outz_h >> 7);
	if(negative)
		outz = (outz | ~((1 << 15) -1));

	ctx->mag_x = outx * 1.5;
	ctx->mag_y = outy * 1.5;
	ctx->mag_z = outz * 1.5;

	XPRINTF("MAG=%d,%d,%d\r\n", ctx->mag_x, ctx->mag_y, ctx->mag_z);
}

double mag_angle(MagnetometerData *ctx)
{
	double angle = (float)180/PI * atan2(ctx->mag_y, ctx->mag_x);

	angle -= calibration_angle;

	angle = (angle < 0) ? angle + 360 : angle;

	XPRINTF("Azimuth wrt magnetic North: %d\r\n", (int)angle);
	return angle;
}

void mag_calibrate(double angle)
{
	calibration_angle = angle;
}

/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata,
										  uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	uint8_t dataReg = (uint8_t)Reg;

	/* CS Enable */
	LSM_MAG_CS_LOW();

	if (BSP_SPI2_Send(&dataReg, 1) != 1)
	{
		ret = BSP_ERROR_UNKNOWN_FAILURE;
	}

	if (BSP_SPI2_Send(pdata, len) != len)
	{
		ret = BSP_ERROR_UNKNOWN_FAILURE;
	}

	/* CS Disable */
	LSM_MAG_CS_HIGH();

	return ret;
}

/**
 * @brief  Read register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be read
 * @param  pdata the pointer to the data to be read
 * @param  len the length of the data to be read
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata,
										 uint16_t len)
{
	int32_t ret = BSP_ERROR_NONE;
	uint8_t dataReg = (uint8_t)Reg;

	/* CS Enable */
	LSM_MAG_CS_LOW();

	//  XPRINTF("Data Read = %d,%d\r\n",(dataReg) | 0x80,pdata[0]);
	LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
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
	LSM_MAG_CS_HIGH();

	SPI_1LINE_TX(&hbusspi2);
	__HAL_SPI_ENABLE(&hbusspi2);
	return ret;
}
