#include "gyroscope.h"

void gyro_init()
{

}

void gyro_read()
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

	BSP_LSM6DSM_ReadReg_Gyro(0x22U, &outx_l, 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x23U, &outx_h, 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x24U, &outy_l, 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x25U, &outy_h, 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x26U, &outz_l, 1);
	BSP_LSM6DSM_ReadReg_Gyro(0x27U, &outz_h, 1);

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

	// XPRINTF("GYRO=%d,%d,%d\r\n", outx, outy, outz);
}