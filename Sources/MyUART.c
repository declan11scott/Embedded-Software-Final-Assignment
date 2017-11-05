/*
 * UART.c
 *
 *  Created on: 1 Aug 2017
 *  Last Modified 8 Aug 2017
 *      Author: 11970744, 11986282
 */
/*!
 **  @addtogroup UART_module packet module documentation
 **  @
 */
/* MODULE UART */

#include "MyUART.h"
#include "Cpu.h"
#include "MK70F12.h"
#include "FIFO.h"
#include "PE_Types.h"

#define THREAD_STACK_SIZE 100

uint8_t MyUART_TempRxData;

OS_ECB* UARTRxSemaphore;
OS_ECB* UARTTxSemaphore;


bool MyUART_Init(const uint32_t baudRate, const uint32_t moduleClk) // 38400, CPU_BUS_CLK_HZ
{
   //Initialise PORTE

   UARTRxSemaphore = OS_SemaphoreCreate(0);
   UARTTxSemaphore = OS_SemaphoreCreate(1);

   SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;		//enable PORT_E
   SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;		//enable UART2 clock
   PORTE_PCR16 |= PORT_PCR_MUX(0x03);      //enable ALT3 for MUX to enable UART2_TX out of PORTE
   PORTE_PCR17 |= PORT_PCR_MUX(0x03);      //enable ALT3 for MUX to enable UART2_RX out of PORTE

   UART2_C1 = 0x00;
   UART2_C2 = 0x00;
   UART2_C2 |= UART_C2_RE_MASK;
   UART2_C2 |= UART_C2_TE_MASK;
   UART2_C2 |= UART_C2_RIE_MASK;			//enable to always receive packets
   UART2_C2 |= UART_C2_TIE_MASK;

   UART2_C4 &= ~UART_C4_BRFA_MASK;
   UART2_C4 |= UART_C4_BRFA(((2 * moduleClk)/baudRate) % 32);

   uint16_t divisor = moduleClk / (16 * baudRate);
   UART2_BDH = (divisor & 0x1F00) >> 8;
   UART2_BDL = (divisor & 0x00FF);

   // Initialise NVIC

   NVICISER1 |= NVIC_ISER_SETENA(1 << 17);
   NVICICPR1 |= NVIC_ICPR_CLRPEND(1 << 17);

   MyFIFO_Init(&RX_FIFO);
   MyFIFO_Init(&TX_FIFO);

   return true;
}

bool MyUART_InChar(uint8_t * const dataPtr)
{
   //OS_SemaphoreSignal(RX_FIFO.ItemsAvailableSemaphore);
   return MyFIFO_Get(&RX_FIFO, dataPtr);
}

bool MyUART_OutChar(const uint8_t data)
{
   //UART2_C2 |= UART_C2_TIE_MASK;
   return  MyFIFO_Put(&TX_FIFO, data);
}

void __attribute__ ((interrupt)) MyUART_ISR(void)
{
   OS_ISREnter();

   uint8_t tempdata;

   /* Determine if data is ready to be sent from the Tower to PC */
   if (UART2_C2 & UART_C2_TIE_MASK)
   {
      if (UART2_S1 & UART_S1_TDRE_MASK)
	  {
	     UART2_C2 &= ~UART_C2_TIE_MASK;
	     (void)OS_SemaphoreSignal(UARTTxSemaphore);
	  }
   }

   /* Determine if data has been sent from the PC to Tower */
   if (UART2_C2 & UART_C2_RIE_MASK)
   {
	  if (UART2_S1 & UART_S1_RDRF_MASK)
	  {
	     MyUART_TempRxData = UART2_D;
	     (void)OS_SemaphoreSignal(UARTRxSemaphore);
	  }
   }
   OS_ISRExit();
}

/* END UART */
/*!
 ** @}
 */
