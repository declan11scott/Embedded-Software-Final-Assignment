/*
 * Mathematics.c
 *
 *  Created on: Oct 27, 2017
 *      Author: Declan
 */
#include "types.h"
#include "analog.h"

float Mathematics_RMS(const int16_t* ValuePtr)
{
  float temp;
  temp = (float)*ValuePtr;
  VoltageInputData.RMS = temp * 0.707;

}

bool Mathematics_FindLargest(const int16_t* ValuePtr)
{
  int16_t* InputValue;
  int16_t x, y;

  int i = (int)ValuePtr;
  int16_t* tempPtr;
  *tempPtr = *ValuePtr;

    x = *InputValue;
    y = *(InputValue + 1);
    // Get first entry and compare
    if (x > y)
    {
      // RMS
      Mathematics_RMS(ValuePtr);
      return true;
    }

  return false;
}


