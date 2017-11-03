/*
 * Calculate.h
 *
 *  Created on: Nov 3, 2017
 *      Author: Declan
 */

#include "types.h"
#include "math.h"

#ifndef SOURCES_CALCULATE_H_
#define SOURCES_CALCULATE_H_

typedef enum
{
  Tariff_1,
  Tariff_2,
  Tariff_3
}TTariff;

float Calculate_PF(float angle);

float Calculate_RMS(int16_t value);

float Calculate_InstantPower(int16_t voltage, int16_t current);

uint16_t Calculate_Power(int16_t voltage, int16_t current, int16_t angle, float pf);

int16_t Calculate_Largest(int16_t* valuePtr);

float Calculate_Energy(float* powerPtr);

#endif /* SOURCES_CALCULATE_H_ */
