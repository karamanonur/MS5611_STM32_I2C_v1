/*
 * MS5611.h
 *
 *  Created on: 24 Kas 2021
 *      Author: Administrator
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#include "main.h"
#include "math.h"
#include "i2c.h"

void MS5611_Init(void);
float MS5611_GetTemp(int osr);
float MS5611_GetPressure(int osr);

#endif /* INC_MS5611_H_ */
