/*
 * median.c
 *
 *    @date 11 Sep 2017
 *    @author 11986282
 *    @author 11970744
 */

// include header files with functions used for Lab4
#include <stdlib.h> /*contains quick sort library*/
 #include "types.h"
 #include "median.h"
 #include "Cpu.h"
 #include "MK70F12.h"

/*! @brief QuickSort
 *
 *
 *
 */
void QuickSort(int16_t array[], const uint32_t size)
{
  /* Array of 0 or 1 elements is already sorted */
  if (size <= 1)
  {
    return;
  }

  /* Use the middle value as the pivot for quicksort */
  int16_t pivot = array[size / 2];

  /* Temporary storage for swapping values */
  int16_t temp;

  /* Indexes from bottom to top and top to bottom */
  int8_t lowI, highI;

  /* Loop through the array to move values less than pivot to the left ‘partition’ and values more than pivot to the right ‘partition’ */
  for (lowI = 0, highI = size - 1; ; lowI++, highI--)
  {
    /* Find the next element to swap into the right ‘partition’ */
    while (array[lowI] < pivot)
    {
      lowI++;
    }

    /* Find the next element to swap into the left ‘partition’ */
    while (pivot < array[highI])
    {
      highI--;
    }

    /* Check */
    if (lowI >= highI)
    {
      break;
    }

    /* Swap values */
    temp = array[lowI];
    array[lowI] = array[highI];
    array[highI] = temp;
  }

  /* Recursive call */
  QuickSort(array, lowI);
  QuickSort(array + lowI, size - lowI);
}

int16_t Median_Filter(const int16_t array[], const uint32_t size)
{
  /* Sort the array into ascending order to find the median */
  int16_t sortedArray[size];

  int i;
  for (i = 0; i < size; i++)
  {
    sortedArray[i] = array[i];
  }

  QuickSort(sortedArray, size);

  /* Check if even */
  if (size % 2 == 0)
  {
    /* If even, median is the average of the two middle elements */
    int16_t x = sortedArray[(size / 2) - 1];
    int16_t y = sortedArray[size / 2];
    return ((x + y) / 2);
  }
  else
  {
    /* If odd, median is the middle element */
    return sortedArray[size / 2];
  }
}


/*! @brief Compares values
 *
 *  @param const void * a pointer to where int a is stored used. First element for comparing in the array
 *  @param const void * a pointer to where int b is stored. Second element for comparing values in the array
 *  @return int the result of the equation int a - int b
 */
// int CompareValues(const void * a, const void * b)
// {
//    return ( *(int*)a - *(int*)b );
// }

//  int16_t Median_Filter(const int16_t array[], const uint32_t size)
//  {
//    int16_t tempArray[size];
//    for (int i = 0; i < size ; i++)
//    {
//      tempArray[i] = array[i];
//    }
//    // sort array of a length up to 1024
//    // have to sort array into ascending order
//    // median is the middle number if numbers are even
//    // median is the average of the two middle sorted numbers
//    qsort(tempArray, size, sizeof(array[0]), CompareValues);

//    int16_t median = 0;

//    if (size % 2 == 0 )
//    {
//      median = (tempArray[size/2] + tempArray[(size/2)-1]) /2;
//    }
//    else
//    {
//      median = tempArray[(size+1)/2];
//    }

//    return median;
//  }

/* END median */
/*!
 ** @}
 */
