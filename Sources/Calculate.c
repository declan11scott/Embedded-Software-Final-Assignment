/*
 * Calculate.c
 *
 *  Created on: Oct 27, 2017
 *      Author: Declan
 */

#include "Calculate.h"

#define ADCConversion 0xCCC

OS_ECB *CalculateInstSemaphore;
OS_ECB *CalculateAvgSemaphore;

float RMSMultiple = 0.707;

float Calculate_PF(float angle)
{
  return cosf(angle);
}

float Calculate_RMS(int16_t value)
{
  return (float)((value * RMSMultiple) / ADCConversion);
}

float Calculate_InstantPower(int16_t voltage, int16_t current)
{
  return (float)((voltage / (float)ADCConversion) * (current / (float)ADCConversion));
}

uint16_t Calculate_Power(int16_t voltage, int16_t current, int16_t angle, float pf)
{
  return (uint16_t)(0xFFFF & (voltage * current * ((uint16_t)pf * 1000)));
}

int16_t Calculate_Largest(int16_t x, int16_t y, int16_t z)
{

  int16_t a, b, c;
  int min = 255;
  uint16_t max = 0xFFFF-255;



  if ((uint16_t)x < min)
    return 0;

  // Ignore if not all values are negative.
  if ((x >> 15) ^ (z >> 15))
    return 0;

  if (x >> 14 && z >> 14 && y >> 14)
  {
    a = x * -1;
    b = y * -1;
    c = z * -1;
    if (b < a)
      {
        if (c > b)
        {
    //      if ((a - c) < 20)
          return y;
        }
      }
  }
  else
  {
    a = x;
    b = y;
    c = z;
  }

  // Get first entry and compare
  if (b > a)
  {
    if (c < b)
    {
//      if ((a - c) < 20)
      return y;
    }
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
