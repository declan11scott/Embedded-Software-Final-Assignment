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
#include "Calculate.h"

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

#define ADCConversion 0xCCC

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
float Power[16];
float Energy, iPower;
uint32union_t EnergykWh;

uint16_t* InputVoltPtr;
uint16_t* InputCurrPtr;
float* PowerPtr;
uint16union_t PowerW;

TTariff Tariff;



//    TAnalogInputData InputData[2] =
//        {
//            Cu
//        };

//-----------------------------------------
// Sine wave set up
//-----------------------------------------
TAnalogOutputData VoltageOutput;
TAnalogOutputData CurrentOutput;

TAnalogInputData VoltageInput;
TAnalogInputData CurrentInput;
int16_t InstantPower;
uint8_t CurrentMaxCount[2], VoltageMaxCount[2], TimeDifference, InputFrequency, VoltageCounter, CurrentCounter;
uint16union_t PIT0Frequency;
uint8_t* CurrentMaxPtr;
uint8_t* VoltageMaxPtr;
float Psi;

int16_t sine[32] =
    {
        0x0000,
        0x0265,
        0x04B3,
        0x06D3,
        0x08B0,
        0x0A37,
        0x0B5A,
        0x0C0D,
        0x0C4A,
        0x0C0D,
        0x0B5A,
        0x0A37,
        0x08B0,
        0x06D3,
        0x04B3,
        0x0265,
        0x0000,
        0xFD9B,
        0xFB4D,
        0xF92D,
        0xF750,
        0xF5C9,
        0xF4A6,
        0xF3F3,
        0xF3B6,
        0xF3F3,
        0xF4A6,
        0xF5C9,
        0xF750,
        0xF92D,
        0xFB4D,
        0xFD9B,

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

int16_t FindLargest(TAnalogInputData InputData)
{
  int16_t largestValue = InputData.InputValues[0];
  int16_t temp;
  for (int i = 0 ;  i < 16 ;  i++)
  {
    if(InputData.InputValues[i + 1] >> 15)
      {
        temp = InputData.InputValues[i + 1] * -1;
      }
    else
      {
        temp =  InputData.InputValues[i + 1];
      }
    if (largestValue < temp)
      largestValue = temp;
  }

return largestValue;

}




static void PIT0Callback(void* args)
{
  // Sample both channels
  Analog_Get(0x00, VoltageInput.InputPtr);
  Analog_Get(0x01, CurrentInput.InputPtr);

  // Calculate instant power for energy calculation
  *PowerPtr = Calculate_InstantPower(*VoltageInput.InputPtr, *CurrentInput.InputPtr);
  PowerPtr++;

  if(Calculate_Largest(VoltageInput.InputPtr))
  {
    VoltageInput.LargestCount[1] = (0xFF) & (uint8_t)(*VoltageInput.InputPtr);
    if(VoltageInput.LargestCount[0] > 0)
    {
      // Calculate the frequency
      (VoltageInput.LargestCount[1] - VoltageInput.LargestCount[0]) * 10 / PIT0Frequency.l;
    }

    VoltageInput.LargestCount[0] = VoltageInput.LargestCount[1];

    // RMS
    VoltageInput.RMS = Calculate_RMS(*VoltageInput.Largest);
  }
  else
    VoltageInput.LargestCount[0]++;

  if(Calculate_Largest(CurrentInput.InputPtr))
  {
    CurrentInput.LargestCount[0] = (0xFF) & (uint8_t)(*CurrentInput.InputPtr);

    // RMS
    CurrentInput.RMS = Calculate_RMS(*CurrentInput.Largest);

    // Phase
    CurrentInput.Phase = (float)(360.0 * (CurrentInput.LargestCount - VoltageInput.LargestCount) * (float)PIT0Frequency.l);

    // Power factor
    VoltageInput.PF = Calculate_PF(CurrentInput.Phase);
  }
  else
    CurrentInput.LargestCount[0]++;

  // Check if the whole sample array is 16
  if(VoltageInput.InputPtr == &VoltageInput.InputValues[15])
  {
    // Reset input value pointers to re-sample.
    VoltageInput.InputPtr = VoltageInput.InputValues;
    CurrentInput.InputPtr = CurrentInput.InputValues;

    // Energy
    PowerPtr = Power;
    Energy += Calculate_Energy(PowerPtr);
    PowerPtr = Power;
  }
  else
  {
    VoltageInput.InputPtr++;
    CurrentInput.InputPtr++;
  }
}


static void PIT1Callback(void* args)
{
  Analog_Put(0x00, *VoltageOutput.wavePtr);
  Analog_Put(0x01, 3 * *CurrentOutput.wavePtr);
  if (VoltageOutput.wavePtr >= &sine[30])
  {
    VoltageOutput.wavePtr = sine;
  }
  else
    {
    VoltageOutput.wavePtr+=2;
    }

  if (VoltageOutput.wavePtr >= &sine[30])
  {
    CurrentOutput.wavePtr = sine;
  }
  else
    {
    CurrentOutput.wavePtr+=2;
    }
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

uint16_t TariffToUFind()
{

}

static void RTC_Callback(void *args)
{
  // Calculate energy for the second and convert to kWh.
  if(Tariff == Tariff_1)
    TariffToUFind();
  Calculate_Cost(uint16 TToU, uint16_t Energy
}

void TestMode()
{
  if(Packet_Parameter1 == 0x01)
  {
    MyPIT_Set(50, PIT_SELECT_1); // assign the period
    MyPIT_Enable(true, PIT_SELECT_1);
  }
  else
    MyPIT_Enable(false, PIT_SELECT_1);
}


void DEMInit()
{

  int phase = 0;
  Packet_Init(UARTBaudRate, CPU_BUS_CLK_HZ);
  Flash_Init();
//  LEDs_Init();
  MyPIT_Init(PIT0Callback,PIT1Callback,PIT2Callback,PIT3Callback, NULL);
  RTC_Init(RTC_Callback, NULL);
//  FTM_Init();
  Analog_Init(CPU_BUS_CLK_HZ);
  
  // 1/period
//  pit2Frequency =

  VoltageOutput.wavePtr = sine;
  CurrentOutput.wavePtr = sine + phase;
  VoltageMaxPtr = VoltageMaxCount;
  CurrentMaxPtr = CurrentMaxCount;

  InputVoltPtr = InputVoltValues;
  InputCurrPtr = InputCurrValues;
  PowerPtr = Power;

  VoltageInput.InputPtr = VoltageInput.InputValues;
  CurrentInput.InputPtr = CurrentInput.InputValues;

  MyPIT_Set(50, PIT_SELECT_0); // assign the period
  MyPIT_Enable(true, PIT_SELECT_0);

  // These two for the DAC when in test mode.
  MyPIT_Set(PIT_DELAY, PIT_SELECT_2); // assign the period
  MyPIT_Enable(true, PIT_SELECT_2);
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

//      case CMD_POWER:
//        uint16union_t Power = Calculate_Power(VoltageInput.RMS, CurrentInput.RMS, CurrentInput.Phase,CurrentInput.PF);
//        Command_Power(Power);
//        break;
//
//      case CMD_ENERGY:
//        Command_Energy(EnergykWh);
//        break;
//
//      case CMD_COST:
//        float cost;
//        Command_Cost((float)cost);
//      break;
//
//      case CMD_FREQUENCY:
//        Command_Frequency((float)VoltageInput.Frequency); //create this command
//      break;
//
//      case CMD_VOLTAGE:
//        Command_Voltage((float)VoltageInput.RMS);
//        break;
//
//      case CMD_CURRENT:
//        Command_Current((float)CurrentInput.RMS);
//      break;
//
//      case CMD_PF:
//        Command_PowerFactor((float)CurrentInput.PF); //create this command
//      break;
//
//      case CMD_FREQ_ENTER:
//        // Frequency * 100
//        PIT0Frequency.l = (float)Packet_Parameter12;
//      break;
//
//      case CMD_VOLTAGE_ENTER:
//        // Voltage * 10
//        VoltageOutput.RMS.l = (int16_t)Packet_Parameter12;
//      break;
//
//      case CMD_CURRENT_ENTER:
//        // Current * 10
//        CurrentOutput.RMS.l = (int16_t)Packet_Parameter12;
//      break;
//
//      case CMD_PHASE_ENTER:
//        // Phase * 1000
//      break;

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
