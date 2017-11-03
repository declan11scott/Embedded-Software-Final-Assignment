/*
 * Mathematics.c
 *
 *  Created on: Oct 27, 2017
 *      Author: Declan
 */
#include "types.h"
#include "analog.h"
#include "Mathematics.h"
#include "math.h"

void Mathematics_RMS(const int16_t* ValuePtr)
{
  float temp;
  temp = (float)*ValuePtr;
  VoltageInputData.RMS = temp * 0.707;

}

void Mathematics_PF(int16_t voltage, int16_t current, int16_t angle)
{
  VoltageInputData.Power = voltage * current * cosf(angle);
}

int16_t Mathematics_FindLargest(int16_t* valuePtr)
{
  int16_t* InputValue = valuePtr;
  int16_t x, y;

  x = *InputValue;
  y = *(InputValue - 1);

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



