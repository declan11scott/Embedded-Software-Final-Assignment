/* ###################################################################
 **     Filename    : main.c
 **     Project     : Project
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"

// Simple OS
#include "OS.h"

// Analog functions
#include "analog.h"

// Packet functions
#include "packet.h"

// Commands
#include "Command.h"

// MK70 and Processor Expert Generated
#include "Cpu.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "MK70F12.h"

#include "FIFO.h"
#include "packet.h"
#include "MyUART.h"
#include "Flash.h"
#include "types.h"
//#include "LEDs.h"
//#include "FTM.h"
//#include "RTC.h"
#include "PIT.h"
#include "analog.h"
#include "median.h"
#include "SPI.h"
#include "OS.h"

// Commands
#define CMD_TEST          0x10
#define CMD_TARIFF        0x11
#define CMD_TIME_A        0x12
#define CMD_TIME_B        0x13
#define CMD_POWER         0x14
#define CMD_ENERGY        0x15
#define CMD_COST          0x16
#define CMD_FREQUENCY     0x17
#define CMD_VOLTAGE       0x18
#define CMD_CURRENT       0x19
#define CMD_POWER_FACTOR  0x20

// Tariff scaled constants by 1000 saved in NvM
#define TARIFF_PEAK       22235
#define TARIFF_SHOULDER   4400
#define TARIFF_OPEAK      2109
#define TARIFF_TWO        1713
#define TARIFF_THREE      4100

// Default constants to be saved in NvM
#define FREQUENCY_Hz      50

// Baudrates and peripheral constants
#define UART_BaudRate     155200
#define PIT_DELAY         100000000

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 4

//      // Thread stacks
//      OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
//      static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//
//      // ----------------------------------------
//      // Thread priorities
//      // 0 = highest priority
//      // ----------------------------------------
//      const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {1, 2, 3, 4};
//
//      /*! @brief Analog thread configuration data
//       *
//       */
//      static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
//      {
//        {
//          .semaphore = NULL,
//          .channelNb = 0
//        },
//        {
//          .semaphore = NULL,
//          .channelNb = 1
//        },
//        {
//          .semaphore = NULL,
//          .channelNb = 2
//        },
//        {
//          .semaphore = NULL,
//          .channelNb = 3
//        },
//      };


void DEMInit()
{
  Packet_Init(UARTBaudRate, CPU_BUS_CLK_HZ);
  Flash_Init();
  LEDs_Init();
  PIT_Init(CPU_BUS_CLK_HZ);
//  RTC_Init();
//  FTM_Init();
  Analog_Init(CPU_BUS_CLK_HZ);

  PIT_Set(100000000, true);
  PIT_Enable(true);

  // Write default tariff values to flash if clear

}

//        /*! @brief Initialises modules.
//         *
//         */
//        static void InitModulesThread(void* pData)
//        {
//          //Initialise Modules
//
//            // Analog
//          (void)Analog_Init(CPU_BUS_CLK_HZ);
//
//          // Generate the global analog semaphores
//          for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
//            AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);
//
//          // We only do this once - therefore delete this thread
//          OS_ThreadDelete(OS_PRIORITY_SELF);
//        }

//      /*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
//       *
//       */
//      void AnalogLoopbackThread(void* pData)
//      {
//        // Make the code easier to read by giving a name to the typecast'ed pointer
//        #define analogData ((TAnalogThreadData*)pData)
//
//        for (;;)
//        {
//          int16_t analogInputValue;
//
//          (void)OS_SemaphoreWait(analogData->semaphore, 0);
//          // Get analog sample
//          Analog_Get(analogData->channelNb, &analogInputValue);
//          // Put analog sample
//          Analog_Put(analogData->channelNb, analogInputValue);
//        }
//      }

/*! @brief Puts the test values through to the DAC
 *
 */
//    void AnalogLoopbackThread(void* pData)
//    {
//      // Make the code easier to read by giving a name to the typecast'ed pointer
//      #define analogData ((TAnalogThreadData*)pData)
//
//      for (;;)
//      {
//        Analog_Get
//      }
//    }

void CommandHandle()
{
  uint16_t a, b, c;
  switch (Packet_Command)
    {
      case CMD_TEST:
        Command_Test(a, b, c);
        break;

      case CMD_TARIFF:
        Command_Tariff(Packet_Parameter1);
        break;

      case CMD_TIME_A:
        Command_TimeA();
        break;

      case CMD_TIME_B:
        Command_TimeB();
        break;

      case CMD_POWER:
        Command_Power();
        break;

      case CMD_ENERGY:
        Command_Energy();
        break;

      case CMD_COST:
        Command_Cost();
      break;

      case CMD_FREQUENCY:
        Command_Frequency(); //create this command
      break;

      case CMD_VOLTAGE:
        Command_Voltage();
        break;

      case CMD_CURRENT:
        Command_Current();
      break;

      case CMD_POWER_FACTOR:
        Command_PowerFactor(); //create this command
      break;

      default:
        break;
    }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
//        OS_ERROR error;
//
//        // Initialise low-level clocks etc using Processor Expert code
//        PE_low_level_init();
//
//        // Initialize the RTOS
//        OS_Init(CPU_CORE_CLK_HZ, true);
//
//        // Create module initialisation thread
//        error = OS_ThreadCreate(InitModulesThread,
//                                NULL,
//                                &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
//                                0); // Highest priority
//
//        // Create threads for analog loopback channels
//        for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
//        {
//          error = OS_ThreadCreate(AnalogLoopbackThread,
//                                  &AnalogThreadData[threadNb],
//                                  &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
//                                  ANALOG_THREAD_PRIORITIES[threadNb]);
//        }
//
//        // Start multithreading - never returns!
//        OS_Start();

  DEMInit();
}

/*!
 ** @}
 */
