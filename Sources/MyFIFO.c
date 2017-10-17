/*
 * FIFO.c
 *
 *  Created on: 1 Aug 2017
 *      @date 8 Aug 2017
 *      @author: 11970744, 11986282
 */
/*!
 **  @addtogroup MyFIFO_module FIFO module documentation
 **  @{
 */
/* MODULE FIFO */

#include "FIFO.h"
#include "Cpu.h"

/*! @brief Initialize the MyFIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void MyFIFO_Init(TFIFO * const FIFO)
{
   FIFO->Start = 3;
   FIFO->End = FIFO->Start;
   FIFO->NbBytes = 0;

   FIFO->SpaceAvailableSemaphore = OS_SemaphoreCreate(FIFO_SIZE);
   FIFO->ItemsAvailableSemaphore = OS_SemaphoreCreate(0);
   FIFO->AccessSemaphore = OS_SemaphoreCreate(1);
}

bool MyFIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
   OS_ERROR error;
   error = OS_SemaphoreWait(FIFO->SpaceAvailableSemaphore, 0);
   error = OS_SemaphoreWait(FIFO->AccessSemaphore, 0);

   OS_DisableInterrupts();

   if (FIFO->NbBytes >= FIFO_SIZE)
     return false;

   FIFO->Buffer[FIFO->End] = data;

   FIFO->End++;
   FIFO->NbBytes++;

   if (FIFO->End >= FIFO_SIZE)
     FIFO->End = 0;

   OS_EnableInterrupts();

   OS_SemaphoreSignal(FIFO->AccessSemaphore);
   OS_SemaphoreSignal(FIFO->ItemsAvailableSemaphore);

   //unlock bufferaccess
   //signal itemsavailable

   return true;
}

bool MyFIFO_Get(TFIFO * const FIFO, uint8_t volatile * const dataPtr)
{
   //wait for itemsAvailable

   OS_ERROR error;
   error = OS_SemaphoreWait(FIFO->ItemsAvailableSemaphore, 0);
   error = OS_SemaphoreWait(FIFO->AccessSemaphore, 0);

   OS_DisableInterrupts();

   if (FIFO->NbBytes == 0)
     return false;

   *dataPtr = FIFO->Buffer[FIFO->Start];

   FIFO->Start++;
   FIFO->NbBytes--;

   if (FIFO->Start >= FIFO_SIZE)
     FIFO->Start = 0;

   OS_EnableInterrupts();

   OS_SemaphoreSignal(FIFO->AccessSemaphore);
   OS_SemaphoreSignal(FIFO->SpaceAvailableSemaphore);

   return true;
}

/* END FIFO */
/*!
 ** @}
 */
