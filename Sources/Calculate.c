/*
 * Calculate.c
 *
 *  Created on: Oct 27, 2017
 *      Author: Declan
 */

#include "Calculate.h"

#define ADCConversion 0xCCC

float RMSMultiple = 0.707;

float Calculate_PF(float angle)
{
  return cosf(angle);
}

float Calculate_RMS(int16_t value)
{
  return value / ADCConversion * RMSMultiple;
}

float Calculate_InstantPower(int16_t voltage, int16_t current)
{
  return (float)voltage / ADCConversion * (float)current / ADCConversion;
}

uint16_t Calculate_Power(int16_t voltage, int16_t current, int16_t angle, float pf)
{
  return (uint16_t)(0xFFFF & (voltage * current * ((uint16_t)pf * 1000)));
}

int16_t Calculate_Largest(int16_t* valuePtr)
{
  int16_t* InputValue = valuePtr;
  int16_t x, y;

  x = *InputValue;
  y = *(InputValue - 1);

  // Ignore negative values.
  if ((x | y) >> 15)
    return 0;

  // Get first entry and compare
  if (x > y)
  {
    // RMS

    return *InputValue;
  }

  return 0;
}

float Calculate_Energy(float* powerPtr)
{
  float energy;
  int i = (int)powerPtr;
  //energy = sum of (Power*Ts)
  for (i = 0; i < 16; i++)
  {
    energy += *powerPtr * 0.125;
    powerPtr++;
  }
  return energy;
}
