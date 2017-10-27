/*
 * Mathematics.c
 *
 *  Created on: Oct 27, 2017
 *      Author: Declan
 */
#include "types.h"
#include "analog.h"

int16_t* Mathematics_FindLargest(const int16_t* ValuePtr, const int sampleSize, const bool VorI)
{
  int16_t* InputValue;
  int16_t x, y;
  if(VorI == 1)
  {
    InputValue = InputVoltValues;
  }
  else
  {
    InputValue = InputCurrValues;
  }
  int i = (int)ValuePtr;
  int16_t* tempPtr;
  *tempPtr = *ValuePtr;
  for(*tempPtr = 0; *tempPtr < sampleSize; *tempPtr++)
  {
    x = *InputValue;
    y = *(InputValue + 1);
    // Get first entry and compare
    if (x > y)
    {
      break; // The for loop, largest found.
    }
  }
  return tempPtr;
}


float Mathematics_RMS(const int16_t* ValuePtr, const int sampleSize, const bool VorI)
{
  Mathematics_FindLargest(ValuePtr, sampleSize, VorI);
//          // Do a quick calc.
//          long i;
//          float x2, y;
//          const threeHalves = 1.5F;
//
//          x2 = Vp * 0.5F;
//          y = (float)number;
//          i = * (long*) &y;
//          i = 0x5F3759DF - (i>>1);
//          y = *(float*) &i;
//          y =
  float temp;
  temp = (float)*ValuePtr;
  return temp * 0.707;

}
