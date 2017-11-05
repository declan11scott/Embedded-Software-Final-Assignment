/*
 * Flash.c
 *
 *  Created on: 15 Aug 2017
 *      Author: 11970744, 11986282
 */

#include "types.h"
#include "Flash.h"
#include "MK70F12.h"

//Block 2, sector 2 address is 0x0008_0000;



uint64union_t virtualRAM;
uint8_t ArrayIndex[10];



/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */
bool MyFlash_AllocateVar(volatile void** variable, const uint8_t size){
  // Number of unallocated bytes
    uint8_t availableSpace = 0;
    //Location of those unallocated bytes from above
    uint8_t locationOfSpace = 0;

    //loop the loop in the data map
    for (uint8_t i = 0; i < 10; i++)
      {
        // if this byte is unallocated
        if (ArrayIndex[i] == 0)
    {
      // remember how many consecutive bytes are unallocated, you might need them later
      availableSpace++;
    }
        else
    {
      // as soon as you find some stored info you gotta start from zero again
      availableSpace = 0;
    }
        // if we can put this byte/half word/word/phrase in this location then be happy with that location
        if (i % size == 0)
    {
      // remember the location of where you might want to store the data
      locationOfSpace = i;
    }
        // if the available space is large enough, and if alignment of bytes is ok
        if (availableSpace >= size && ((i + 1) % size == 0))
    {
      for (uint8_t j = locationOfSpace; j <= i; j++)
        {
          // mark these as allocated
          ArrayIndex[j] = 1;
        }
      // the address start is the first byte of what was allocated
      *variable = (void*) (FLASH_DATA_START + locationOfSpace);

      // data has been allocated space
      return true;
    }
      }
    // if unable to allocate space, you failed
    return false;
}

