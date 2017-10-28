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
#include "UART.h"
#include "Flash.h"
#include "types.h"
//#include "LEDs.h"
//#include "FTM.h"
#include "RTC.h"
#include "PIT.h"
#include "analog.h"
#include "median.h"
//#include "SPI.h"
#include "OS.h"
#include "Command.h"

// Tariff scaled constants by 1000 saved in NvM
#define TARIFF_PEAK       22235
#define TARIFF_SHOULDER   4400
#define TARIFF_OPEAK      2109
#define TARIFF_TWO        1713
#define TARIFF_THREE      4100

// Default constants to be saved in NvM
#define FREQUENCY_Hz      50

// Baudrates and peripheral constants
#define UARTBaudRate     115200
#define PIT_DELAY        10000000

// Flash defines

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100

    int16_t InputVoltValues[16];
    int16_t InputCurrValues[16];
    float   VoltageRMS[ANALOG_NB_IO];
    float   CurrentRMS[ANALOG_NB_IO];

    uint16_t* InputVoltPtr;
    uint16_t* InputCurrPtr;

//    TAnalogInputData InputData[2] =
//        {
//            Cu
//        };

//-----------------------------------------
// Sine wave set up
//-----------------------------------------
TAnalogOutputData AnalogOutput[2];

TAnalogInputData VoltageInputData;
TAnalogInputData CurrentInputData;
int32_t InstantPower;

int16_t sine[18] =
    {
        0xFC0C,
        0xF81F,
        0xF533,
        0xF393,
        0xF3B1,
        0xF53B,
        0xF7DF,
        0xFB60,
        0xFF47,
        0x0377,
        0x0719,
        0x09F8,
        0x0BC5,
        0x0C4A,
        0x0AE9,
        0x083C,
        0x0480,
        0x0034
    };


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

static void PIT0Callback(void* args)
{
//      uint16union_t flashTemp;
//      *Flashy = FLASH_DATA_START;
//        Analog_Put(0x00, *AnalogOutput.wavePtr);
//
//
//        if (*AnalogOutput.wavePtr == AnalogOutput[i])
//          wavePtr = sine;
//        else wavePtr++;

//      //read and send back flash data.
//      for(uint16_t i = 0; i < 8; i++)
//      {
//        flashTemp.l = _FH(Flashy);
//        Packet_Put(0x40, 0x00, flashTemp.s.Lo, flashTemp.s.Hi);
//        Flashy++;
//      }

  //sample the analog data and send to computer

  Analog_Get(0x00, VoltageInputData.InputPtr);
  Analog_Get(0x01, CurrentInputData.InputPtr);
  VoltageInputData.InputPtr++;
  CurrentInputData.InputPtr++;

  InstantPower = (uint32_t)(VoltageInputData.InputPtr * CurrentInputData.InputPtr);

  if(Mathematics_FindLargest(InputVoltPtr))
    // Calculate the amount of interrupts to find frequency and p.f
    ;
  if(Mathematics_FindLargest(InputCurrPtr))
    // Calculate the amount of interrupts to find frequency and p.f
    ;
  // Calculate instantaneous power
  // Calculate cost
  if(VoltageInputData.InputPtr == &VoltageInputData.InputValues[15])
    // Call the functions to calc.
    //
    // - Multiply with current and store in the a p array
    // - calc. cost
    // - clear Ptrs
  {

  }
}


static void PIT1Callback(void* args)
{
  Analog_Put(0x00, *AnalogOutput[0].wavePtr);
  Analog_Put(0x01, *AnalogOutput[1].wavePtr);
  if (AnalogOutput[0].wavePtr == &sine[17])
    AnalogOutput[0].wavePtr = sine;
  else AnalogOutput[0].wavePtr++;
}

static void PIT2Callback(void* args)
{

}

static void PIT3Callback(void* args)
{

}

void TariffDefaults()
{
  bool success;
  uint16_t* address;
  uint8_t size = 0x02;
  // Read the values, if different, write defaults.
  if (_FH(FLASH_DATA_START) != TARIFF_THREE)
  {
    // If different, erase all before writing
    success = Flash_Erase();
    Flash_AllocateVar((volatile void**)&address, size);
    address+2;
    success &= Flash_Write16(address,TARIFF_PEAK);
    address+2;
    success &= Flash_Write16(address,TARIFF_SHOULDER);
    address+2;
    success &= Flash_Write16(address,TARIFF_OPEAK);
    address+2;
    success &= Flash_Write16(address,TARIFF_TWO);
    address+2;
    success &= Flash_Write16(address,TARIFF_THREE);
  }
}

static void RTC_Callback(void *args)
{

}


void DEMInit()
{
  Packet_Init(UARTBaudRate, CPU_BUS_CLK_HZ);
  Flash_Init();
//  LEDs_Init();
  MyPIT_Init(PIT0Callback,PIT1Callback,PIT2Callback,PIT3Callback, NULL);
  RTC_Init(RTC_Callback, NULL);
//  FTM_Init();
  Analog_Init(CPU_BUS_CLK_HZ);
  
// Initialise the wave pointers.
  for(int i = 0; i < 2; i++)
  {
//    *AnalogOutput[i].wave = *sine;
    AnalogOutput[i].wavePtr = sine;
//    AnalogInput[i].putPtr  = AnalogInput[i].values;
  }
  InputVoltPtr = InputVoltValues;
  InputCurrPtr = InputCurrValues;

  VoltageInputData.InputPtr = VoltageInputData.InputValues;
  CurrentInputData.InputPtr = CurrentInputData.InputValues;

  MyPIT_Set(PIT_DELAY, PIT_SELECT_0); // assign the period
  MyPIT_Enable(true, PIT_SELECT_0);

  MyPIT_Set(PIT_DELAY, PIT_SELECT_1); // assign the period
  MyPIT_Enable(true, PIT_SELECT_1);

  // These two for the DAC when in test mode.
//  MyPIT_Set(PIT_DELAY, PIT_SELECT_2); // assign the period
//  MyPIT_Enable(true, PIT_SELECT_2);
//
//  MyPIT_Set(PIT_DELAY, PIT_SELECT_3); // assign the period
//  MyPIT_Enable(true, PIT_SELECT_3);

  // Write default tariff values to flash if clear
TariffDefaults();

  // write start up values
  Packet_Put(0x00,0x01,0x02,0x03);
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
      case 0x04:  //remove after testing
        Packet_Put(0x04,0x00,0x00,0x00);
        break;

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
EnterCritical();
  DEMInit();
ExitCritical();


  for(;;)
  {
    if (Packet_Get())
    {
//      FTM_StartTimer(&aFTMChannel);
//      LEDs_On(LED_BLUE);
      CommandHandle();
  }
}
}

/*!
 ** @}
 */
