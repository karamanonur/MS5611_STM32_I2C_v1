#include "MS5611.h"

/* Device address */
static uint8_t MS5611_DEVICE_ADDR = 0x77;

/* Commands */
static uint8_t MS5611_RESET = 0x1E;

static uint8_t D1_OSR_256   = 0x40;
static uint8_t D1_OSR_512   = 0x42;
static uint8_t D1_OSR_1024  = 0x44;
static uint8_t D1_OSR_2048  = 0x46;
static uint8_t D1_OSR_4096  = 0x48;

static uint8_t D2_OSR_256   = 0x50;
static uint8_t D2_OSR_512   = 0x52;
static uint8_t D2_OSR_1024  = 0x54;
static uint8_t D2_OSR_2048  = 0x56;
static uint8_t D2_OSR_4096  = 0x58;

static uint8_t PROM_READ_C1 = 0xA2;
static uint8_t PROM_READ_C2 = 0xA4;
static uint8_t PROM_READ_C3 = 0xA6;
static uint8_t PROM_READ_C4 = 0xA8;
static uint8_t PROM_READ_C5 = 0xAA;
static uint8_t PROM_READ_C6 = 0xAC;

static uint8_t ADC_READ     = 0x00;

/* Calibration data to be read from PROM */
static uint16_t C1 = 0;	// Pressure sensitivity | SENS(T1)
static uint16_t C2 = 0;	// Pressure offset | OFF(T1)
static uint16_t C3 = 0;	// Temperature coefficient of pressure sensitivity | TCS
static uint16_t C4 = 0;	// Temperature coefficient of pressure offset | TCO
static uint16_t C5 = 0;	// Reference temperature | T(REF)
static uint16_t C6 = 0;	// Temperature coefficient of the temperature | TEMPSENS

/* Digital pressure and temperature data to be read */
static uint32_t D1 = 0;	// Digital pressure value
static uint32_t D2 = 0;	// Digital temperature value

/* Variables to calculate actual temperature */
static int32_t dT   = 0; // Difference between actual and reference temperature
static int32_t TEMP = 0; // Actual temperature

/* Variables to calculate temperature compensated pressure */
static int64_t OFF  = 0; // Offset at actual temperature
static int64_t SENS = 0; // Sensitivity at actual temperature
static int64_t P    = 0; // Temperature compensated pressure

/* Reset function */
static void MS5611_Reset(void)
{
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MS5611_DEVICE_ADDR<<1), &MS5611_RESET, 1, 50);
}

/* Function to read pre-calibrated data */
static void MS5611_ReadCalibrationData(void)
{
	uint8_t calibrationDataBuff[2] = {0};

	/* Setting PROM read mode and reading calibration datas */

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C1, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calibrationDataBuff, 2, 50);
	C1 = ((calibrationDataBuff[0] << 8) | (calibrationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C2, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calibrationDataBuff, 2, 50);
	C2 = ((calibrationDataBuff[0] << 8) | (calibrationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C3, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calibrationDataBuff, 2, 50);
	C3 = ((calibrationDataBuff[0] << 8) | (calibrationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1,(MS5611_DEVICE_ADDR<<1), &PROM_READ_C4, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calibrationDataBuff, 2, 50);
	C4 = ((calibrationDataBuff[0] << 8) | (calibrationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C5, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calibrationDataBuff, 2, 50);
	C5 = ((calibrationDataBuff[0] << 8) | (calibrationDataBuff[1]));

	HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &PROM_READ_C6, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), calibrationDataBuff, 2, 50);
	C6 = ((calibrationDataBuff[0] << 8) | (calibrationDataBuff[1]));
}

/* Function to read digital temperature and pressure values according to desired sampling rate */
static void MS5611_ReadDigitalValues(int osr)
{
	uint8_t digitalPressureBuff[3] = {0};
	uint8_t digitalTempBuff[3] = {0};

	switch(osr)
	{
	case 256:
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_256, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 10);
		D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_256, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 10);
		D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
		break;
	case 512:
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_512, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 10);
		D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_512, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 10);
		D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
		break;
	case 1024:
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_1024, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 10);
		D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_1024, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 10);
		D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
		break;
	case 2048:
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_2048, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 10);
		D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_2048, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 10);
		D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
		break;
	case 4096:
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_4096, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 10);
		D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_4096, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 10);
		D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
		break;
	default:
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D1_OSR_1024, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalPressureBuff, 3, 10);
		D1 = ((digitalPressureBuff[0] << 16) | (digitalPressureBuff[1] << 8) | (digitalPressureBuff[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &D2_OSR_1024, 1, 10);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_DEVICE_ADDR<<1), &ADC_READ, 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)((MS5611_DEVICE_ADDR<<1)|0x01), digitalTempBuff, 3, 10);
		D2 = ((digitalTempBuff[0] << 16) | (digitalTempBuff[1] << 8) | (digitalTempBuff[2]));
		break;
	}
}

static void MS5611_DoCalculations(void)
{
	/* For dT, OFF, and SENS max and min values should be defined */
	dT = D2 - (C5 * pow(2,8));
	if(dT < -16776960) dT = -16776960;
	if(dT > 16777216) dT = 16777216;

	TEMP = 2000 + (dT * C6 /pow(2,26));

	OFF = (C2 * pow(2,16)) + ((C4 * dT) / pow(2,7));
	if(OFF < -8589672450) OFF = -8589672450;
	if(OFF > 12884705280) OFF = 12884705280;

	SENS = (C1 * pow(2,15)) + ((C3 * dT) / pow(2,8));
	if(SENS < -4294836225) SENS = -4294836225;
	if(SENS > 6442352640) SENS = 6442352640;

	P = ((D1 * SENS / pow(2,21) - OFF)/ pow(2,15));
}

/* Function to get actual pressure value */
float MS5611_GetPressure(int osr)
{
	float actualPressure = 0;

	MS5611_ReadDigitalValues(osr);
	MS5611_DoCalculations();

	actualPressure = P / 100.00;
	return actualPressure;
}

/* Function to get actual temperature value */
float MS5611_GetTemperature(int osr)
{
	float actualTemperature = 0;

	MS5611_ReadDigitalValues(osr);
	MS5611_DoCalculations();

	actualTemperature = TEMP / 100.00;
	return actualTemperature;
}

void MS5611_Init(void)
{
	MS5611_Reset();
	HAL_Delay(50);
	MS5611_ReadCalibrationData();
}
