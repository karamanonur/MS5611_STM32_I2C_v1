#include "MS5611.h"

//extern I2C_HandleTypeDef hi2c1;

/* Addresses */
uint8_t MS5611_DEVICE_ADDR = 0x77;

/* Commands */
uint8_t MS5611_RESET	= 0x1E;

uint8_t D1_OSR_256 		= 0x40;
uint8_t D1_OSR_512 		= 0x42;
uint8_t D1_OSR_1024 	= 0x44;
uint8_t D1_OSR_2048 	= 0x46;
uint8_t D1_OSR_4096 	= 0x48;

uint8_t D2_OSR_256 		= 0x50;
uint8_t D2_OSR_512 		= 0x52;
uint8_t D2_OSR_1024 	= 0x54;
uint8_t D2_OSR_2048 	= 0x56;
uint8_t D2_OSR_4096		= 0x58;

uint8_t PROM_READ_C1 	= 0xA2;
uint8_t PROM_READ_C2 	= 0xA4;
uint8_t PROM_READ_C3 	= 0xA6;
uint8_t PROM_READ_C4	= 0xA8;
uint8_t PROM_READ_C5 	= 0xAA;
uint8_t PROM_READ_C6 	= 0xAC;

uint8_t ADC_READ 		= 0x00;

/* Calibration data to be read from PROM */
uint16_t C1 = 0;	// Pressure sensitivity | SENS(T1)
uint16_t C2 = 0;	// Pressure offset | OFF(T1)
uint16_t C3 = 0;	// Temperature coefficient of pressure sensitivity | TCS
uint16_t C4 = 0;	// Temperature coefficient of pressure offset | TCO
uint16_t C5 = 0;	// Reference temperature | T(REF)
uint16_t C6 = 0;	// Temperature coefficient of the temperature | TEMPSENS

/* Digital pressure and temperature data to be read */
uint32_t D1 = 0;	// Digital pressure value
uint32_t D2 = 0;	// Digital temperature value

/* Variables to calculate actual temperature */
int32_t dT   = 0;	// Difference between actual and reference temperature
int32_t TEMP = 0;	// Actual temperature

/* Variables to calculate temperature compensated pressure */
int64_t OFF  = 0;	// Offset at actual temperature
int64_t SENS = 0;	// Sensitivity at actual temperature
int64_t P    = 0;	// Temperature compensated pressure

/* Actual values */
float actualTemperature = 0;
float actualPressure = 0;

//static void MS5611_Reset(void)
//{
//	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MS5611_DEVICE_ADDR<<1), &MS5611_RESET, 1, 50);
//}

static void MS5611_ReadCalibrationData(void)
{
	uint8_t calirationDataBuff[2] = {0};

	/* Setting PROM read mode and reading calibration datas */

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C1, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calirationDataBuff, 2, 50);
	C1 = ((calirationDataBuff[0] << 8) | (calirationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C2, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calirationDataBuff, 2, 50);
	C2 = ((calirationDataBuff[0] << 8) | (calirationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C3, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calirationDataBuff, 2, 50);
	C3 = ((calirationDataBuff[0] << 8) | (calirationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1,(MS5611_DEVICE_ADDR<<1), &PROM_READ_C4, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calirationDataBuff, 2, 50);
	C4 = ((calirationDataBuff[0] << 8) | (calirationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C5, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calirationDataBuff, 2, 50);
	C5 = ((calirationDataBuff[0] << 8) | (calirationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C6, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calirationDataBuff, 2, 50);
	C6 = ((calirationDataBuff[0] << 8) | (calirationDataBuff[1]));
}

static void MS5611_ReadDigitalValues(int osr)
{
	uint8_t digitalPressureBuff[3] = {0};
	uint8_t digitalTempBuff[3] = {0};

	switch(osr)
	{
		case 256:
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_256, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 50);
			D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_256, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 50);
			D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
			break;
		case 512:
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_512, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 50);
			D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_512, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 50);
			D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
			break;
		case 1024:
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_1024, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 50);
			D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_1024, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 50);
			D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
			break;
		case 2048:
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_2048, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 50);
			D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_2048, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 50);
			D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
			break;
		case 4096:
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_4096, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 50);
			D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_4096, 1, 50);
			HAL_Delay(1000);
			HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 50);
			HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 50);
			D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
			break;
		default:
			break;
	}
}

float MS5611_GetPressure(int osr)
{
	/* Calculate and return actual pressure */

	MS5611_ReadDigitalValues(osr);

	OFF = (C2 * pow(2,16)) + ((C4 * dT) / pow(2,7));
	SENS = (C1 * pow(2,15)) + ((C3 * dT) / pow(2,8));
	P = ((D1 * SENS / pow(2,21) - OFF)/ pow(2,15));
	actualPressure = P / 100.00;

	return actualPressure;
}

float MS5611_GetTemp(int osr)
{
	/* Calculate and return actual temperature */

	MS5611_ReadDigitalValues(osr);

	dT = D2 - (C5 * pow(2,8));
	TEMP = 2000 + (dT * C6 /pow(2,26));
	actualTemperature = TEMP / 100.00;

	return actualTemperature;
}

void MS5611_Init(void)
{
	MS5611_ReadCalibrationData();
}
