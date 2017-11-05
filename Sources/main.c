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
#include "RTC.h"
#include "PIT.h"
#include "analog.h"
//#include "SPI.h"
#include "OS.h"
#include "Command.h"
#include "Calculate.h"
#include "Tariff.h"

// Default constants to be saved in NvM
#define FREQUENCY_Hz      50

// Baudrates and peripheral constants
#define UARTBaudRate     115200
#define PIT_DELAY        10000000

#define ADCConversion 0xCCC


// Tariff scaled constants by 1000 saved in NvM
#define TARIFF_PEAK       22235
#define TARIFF_SHOULDER   4400
#define TARIFF_OPEAK      2109
#define TARIFF_TWO        1713
#define TARIFF_THREE      4100


// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100

float Power[16];
float Energy, iPower, iVoltage, iCurrent;
uint32union_t EnergykWh;

uint16_t* InputVoltPtr;
uint16_t* InputCurrPtr;
float* PowerPtr;
uint16union_t PowerW;

TTariff Tariff;

OS_ECB* FlashSemaphore;


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
uint8_t CurrentMaxCount, VoltageMaxCount[2], TimeDifference, InputFrequency, VoltageCounter, CurrentCounter;
uint16union_t PIT0Frequency;
uint8_t CurrentMaxCtr;
uint8_t VoltageMaxCtr;
float Cost;
float Psi;
int Phase = 0;


// Flash variables
static volatile uint16union_t* NvTariff1Peak;
static volatile uint16union_t* NvTariff1Shoulder;
static volatile uint16union_t* NvTariff1OPeak;
static volatile uint16union_t* NvTariff2;
static volatile uint16union_t* NvTariff3;

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
      OS_ECB* RTCSemaphore;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[ANALOG_NB_IO][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PIT0ThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PIT1ThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t RTCThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FlashThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[ANALOG_NB_OUTPUTS] = {2, 3};

/*! @brief Analog thread configuration data
 *
 */
TAnalogThreadData AnalogThreadData[ANALOG_NB_IO] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  },
};

int16_t FindLargest(TAnalogInputData InputData)
{

//    OS_Wait(CalculateSemaphore,0);
    int16_t largestValue = InputData.InputValues[0];
    int16_t temp;
    int min = 255;
    uint16_t max = 0xFFFF-255;

    for (int i = 0 ;  i < 15 ;  i++)
    {
      if (!(uint16_t)largestValue < min | (uint16_t)largestValue > max)
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
    }
 return largestValue;
}

float MeasureEnergy()
{
  float sampleEnergy;
  for (int i = 0; i < 16; i++)
  {
    sampleEnergy += *PowerPtr * 0.125 / 1000000;
  }
  return sampleEnergy;
}
static void CalculateCost()
{

}

static void PIT0Callback(void* args)
{
  for(;;)
  {
    OS_SemaphoreWait(PIT0Semaphore, 0);
    // Sample both channels
    Analog_Get(0x00, VoltageInput.InputPtr);
    Analog_Get(0x01, CurrentInput.InputPtr);

    // Calculate instant power for energy calculation
  //  *PowerPtr = Calculate_InstantPower(*VoltageInput.InputPtr, *CurrentInput.InputPtr);
  //
        iVoltage = (float)(*VoltageInput.InputPtr/3.3);
        iCurrent = (float)(*CurrentInput.InputPtr/3.3);
        *PowerPtr = iVoltage * iCurrent;
        *PowerPtr++;
  //        int16_t power = ((int32_t)*VoltageInput.InputPtr * (int32_t)*CurrentInput.InputPtr) >> 16;

//            if(Calculate_Largest(*VoltageInput.InputPtr, *(VoltageInput.InputPtr - 1), *(VoltageInput.InputPtr - 2)))
//            {
//              if(VoltageMaxCount[0] > 0)
//                {
//                  VoltageMaxCount[1] = VoltageMaxCtr;
//                  VoltageInput.Frequency = (50 * 16) / (2 * (VoltageMaxCount[1] - VoltageMaxCount[0])) ;
//                }
//              else VoltageMaxCount[0] = VoltageMaxCtr;
//
//              VoltageInput.LargestCount[1] = (0xFF) & (uint8_t)(*VoltageInput.InputPtr);
//              if(VoltageInput.LargestCount[0] > 0)
//              {
//                // Calculate the frequency
//          //      VoltageInput.Frequency = (50 * 16) / (VoltageInput.LargestCount[1] - VoltageInput.LargestCount[0]) ;
//              }
//
//              VoltageInput.LargestCount[0] = VoltageInput.LargestCount[1];
//
//              // RMS
//              VoltageInput.RMS = Calculate_RMS(*(VoltageInput.InputPtr - 1));
//
//              if(VoltageMaxCount[0] && CurrentMaxCount)
//              {
//              // Phase
//          //    CurrentInput.Phase = (float)(360 / (CurrentInput.LargestCount[0] - VoltageInput.LargestCount[0]));
//              CurrentInput.Phase = 180 - ((360 * (float)(VoltageMaxCount[0] - CurrentMaxCount + 1)) / 16);
//
//              // Power factor
//              VoltageInput.PF = 100 * Calculate_PF(CurrentInput.Phase);
//              }
//            }
//            else
//            {
//              VoltageInput.LargestCount[0]++;
//            }
//            VoltageMaxCtr++;
//            if(*CurrentInput.InputPtr > 0)
//            {
//              if(Calculate_Largest(*CurrentInput.InputPtr, *(CurrentInput.InputPtr - 1), *(CurrentInput.InputPtr - 2)))
//              {
//                CurrentInput.LargestCount[0] = (0xFF) & (uint8_t)(*CurrentInput.InputPtr);
//                CurrentMaxCount = CurrentMaxCtr;
//                // RMS
//                CurrentInput.RMS = Calculate_RMS(*(CurrentInput.InputPtr - 1));
//
//              }
//              else
//              {
//                CurrentInput.LargestCount[0]++;
//              }
//              CurrentMaxCtr++;
//            }

    // Check if the whole sample array is 16
    if(VoltageInput.InputPtr == &VoltageInput.InputValues[15])
    {
      // Reset input value pointers to re-sample.
      VoltageInput.InputPtr = VoltageInput.InputValues;
      CurrentInput.InputPtr = CurrentInput.InputValues;

      VoltageMaxCtr = 0;
      CurrentMaxCtr = 0;
      VoltageMaxCount[0] = 0;
      VoltageMaxCount[1] = 0;

        VoltageInput.Largest = FindLargest(VoltageInput);
        VoltageInput.RMS = Calculate_RMS(VoltageInput.Largest);

        CurrentInput.Largest = FindLargest(CurrentInput);
        CurrentInput.RMS = Calculate_RMS(CurrentInput.Largest);

      // Energy
      PowerPtr = Power;
      Energy += MeasureEnergy();
      PowerPtr = Power;
    }
    else
    {
      VoltageInput.InputPtr++;
      CurrentInput.InputPtr++;
    }
  }
}


static void PIT1Callback(void* args)
{
  for(;;)
  {
    OS_SemaphoreWait(PIT1Semaphore,0);
    Analog_Put(0x00, *VoltageOutput.wavePtr);
    Analog_Put(0x01, *CurrentOutput.wavePtr);
    if (VoltageOutput.wavePtr > &sine[29])
    {
      VoltageOutput.wavePtr = sine;
    }
    else
      {
      VoltageOutput.wavePtr+=2;
      }

    if (CurrentOutput.wavePtr > &sine[29])
    {
      CurrentOutput.wavePtr = sine;
    }
    else
      {
      CurrentOutput.wavePtr+=2;
      }
  }
}

static void PIT2Callback(void* args)
{

}

static void PIT3Callback(void* args)
{

}

void TariffDefaults(void* args)
{
  bool success;
    int i;
    volatile uint16union_t* Nv[5] =
        {
            NvTariff1Peak, NvTariff1Shoulder, NvTariff1OPeak, NvTariff2, NvTariff3
        };
    int def[5] =
        {
            TARIFF_PEAK, TARIFF_SHOULDER, TARIFF_OPEAK, TARIFF_TWO, TARIFF_THREE
        };

    for(i = 0; i < 5; i++)
    {
    /* Allocate memory in flash will return true if successful*/
        success = MyFlash_AllocateVar((volatile void**)&Nv[i], sizeof(*Nv[i]));

        /* Check if there's no previously set value in NvTowerNb in flash memory*/
        if (success && (*Nv[i]).l == 0xFFFF)
        {
           /* If it's empty, just allocate it the default value which is our student number*/
           success &= Flash_Write16((uint16_t*)Nv[i], def[i]);
        }
    }
     /* Initialise flash just because */
     uint64_t initialiseFlash = _FP(FLASH_DATA_START);

     NvTariff1Peak      = Nv[4];
     NvTariff1Shoulder  = Nv[3];
     NvTariff1OPeak     = Nv[2];
     NvTariff2          = Nv[1];
     NvTariff3          = Nv[0];

    OS_ThreadDelete(OS_PRIORITY_SELF);
}

uint16_t TariffToUFind()
{
  uint16_t aTariff;
  uint8_t hours;
  RTC_Get(&hours, NULL, NULL);
  if(!(hours % 7))
  {
    if(!(hours % 14))
      // Peak
      aTariff = _FH(NvTariff1Peak);
    else aTariff = _FH(NvTariff1Shoulder);
  }
  if(!(hours % 20))
  {
    aTariff = _FH(NvTariff1Shoulder);
  }
  else aTariff = _FH(NvTariff1OPeak);

}

static void RTCThread(void* args)
{
  for(;;)
  {
    OS_SemaphoreWait(RTCSemaphore,0);
    uint16_t aTariff;
    // Calculate energy for the second and convert to kWh.
    switch (Tariff)
    {
      case Tariff_1:
        aTariff = TariffToUFind();
        break;
      case Tariff_2:
        // Load tariff 2 cost
        aTariff = _FH(NvTariff2);
        break;
      case Tariff_3:
        // Load tariff 2 cost
        aTariff = _FH(NvTariff3);
        break;

    }
  //  Calculate_Cost(uint16 TToU, uint16_t Energy
    EnergykWh.l = (int)Energy;
    Cost += ((aTariff * Energy) / 1000000);
    Energy = 0;
  }
}

static void RTC_Callback(void *args)
{
  OS_SemaphoreSignal(RTCSemaphore);
}

void TestMode()
{
  if(Packet_Parameter1 == 0x01)
  {
    MyPIT_Set(50, PIT_SELECT_1); // assign the period
    MyPIT_Enable(true, PIT_SELECT_1);
  }
  else
  {
    MyPIT_Enable(false, PIT_SELECT_1);
    Analog_Put(0x00,0);
    Analog_Put(0x01,0);
  }
}


static void DEMInit()
{

  MyPacket_Init(UARTBaudRate, CPU_BUS_CLK_HZ);
  Flash_Init();
//  LEDs_Init();
  MyPIT_Init(PIT0Callback,PIT1Callback,PIT2Callback,PIT3Callback, NULL);
  RTC_Init(RTC_Callback, NULL);
//  FTM_Init();
  Analog_Init(CPU_BUS_CLK_HZ);
  
  // 1/period
//  pit2Frequency =

  VoltageOutput.wavePtr = sine;
  CurrentOutput.wavePtr = sine + Phase;

  CurrentMaxCtr = 0;

  PowerPtr = Power;

  VoltageInput.InputPtr = VoltageInput.InputValues;
  CurrentInput.InputPtr = CurrentInput.InputValues + Phase;

  MyPIT_Set(50, PIT_SELECT_0); // assign the period
  MyPIT_Enable(true, PIT_SELECT_0);

  // Write default tariff values to flash if clear
//  TariffDefaults();
  Analog_Put(0x00,0);
  Analog_Put(0x01,0);
}


void __attribute__ ((interrupt)) PIT0_ISR(void)
{
  OS_ISREnter();
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
//  for (uint8_t analogNb = 0; analogNb < ANALOG_NB_IO; analogNb++)
//      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
  (void)OS_SemaphoreSignal(PIT0Semaphore);
//  (*Callback0)(Arguments);
  OS_ISRExit();
}

void __attribute__ ((interrupt)) PIT1_ISR(void)
{
  OS_ISREnter();
  PIT_TFLG1 |= PIT_TFLG_TIF_MASK;
  (void)OS_SemaphoreSignal(PIT1Semaphore);
//  (*Callback1)(Arguments);
  OS_ISRExit();
}


        /*! @brief Initialises modules.
         *
         */
static void InitModulesThread(void* pData)
{
  DEMInit();

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < ANALOG_NB_IO; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  RTCSemaphore        = OS_SemaphoreCreate(0);
  PIT0Semaphore       = OS_SemaphoreCreate(0);
  PIT1Semaphore       = OS_SemaphoreCreate(0);
  CalculateSemaphore  = OS_SemaphoreCreate(1);



  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}


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
  uint8_t dCost, cCost, hours, seconds, minutes;
  uint16union_t Freqx10, rVoltage, rCurrent;


  switch (Packet_Command)
    {
      case CMD_TEST:
        if(Packet_Parameter2)
        {
//          uint16union_t T1P, T1S, T1O, T2, T3;
//          T1P.l = _FH(NvTariff1Peak);
//          T1S.l = _FH(NvTariff1Shoulder);
//          T1O.l = _FH(NvTariff1OPeak);
//          T2.l = _FH(NvTariff2);
//          T3.l = _FH(NvTariff3);
//          Packet_Put(T1P.s.Lo, T1P.s.Hi,0x00,0x00);
//          Packet_Put(T1S.s.Lo, T1S.s.Hi,0x00,0x00);
//          Packet_Put(T1O.s.Lo, T1O.s.Hi,0x00,0x00);
//          Packet_Put(T2.s.Lo, T2.s.Hi,0x00,0x00);
//          Packet_Put(T3.s.Lo, T3.s.Hi,0x00,0x00);
//          Command_Test();
        }

        else TestMode();
        break;

      case CMD_TARIFF:
        Command_Tariff(Packet_Parameter1);
        break;

      case CMD_TIME_A:
        RTC_Get(&hours, &minutes, &seconds);
        Command_TimeA(seconds, minutes);
        break;

      case CMD_TIME_B:
        RTC_Get(&hours, &minutes, &seconds);
        Command_TimeB(minutes);
        break;

//      case CMD_POWER:
//        uint16union_t Power = Calculate_Power(VoltageInput.RMS, CurrentInput.RMS, CurrentInput.Phase,CurrentInput.PF);
//        Command_Power(Power);
//        break;

      case CMD_ENERGY:
        Command_Energy(EnergykWh);
        break;

      case CMD_COST:
        dCost = (int)Cost % 100;
        cCost = (int)(Cost * 100) % 100;
        Command_Cost(cCost, dCost);
      break;

      case CMD_FREQUENCY:
        Freqx10.l = (int)((VoltageInput.Frequency) * 10);
        Command_Frequency(Freqx10);
      break;

      case CMD_VOLTAGE:
        rVoltage.l = (int)(VoltageInput.RMS * 100);
        Command_Voltage(rVoltage);
        break;

      case CMD_CURRENT:
        rCurrent.l = (int)(VoltageInput.RMS * 100000);
        Command_Current(rCurrent);
      break;

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

static void PacketThread(void* arg)
{
   for(;;)
   {
     if (MyPacket_Get())
   {

       CommandHandle();
     }
   }
}


/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
        OS_ERROR error;

        // Initialise low-level clocks etc using Processor Expert code
        PE_low_level_init();

        // Initialize the RTOS
        OS_Init(CPU_CORE_CLK_HZ, false);

        // Create module initialisation thread
        error = OS_ThreadCreate(InitModulesThread,
                                NULL,
                                &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                                0); // Highest priority
        error = OS_ThreadCreate(PIT0Callback,
                                &PIT0Semaphore,
                                &PIT0ThreadStacks[THREAD_STACK_SIZE - 1],
                                4);
        error = OS_ThreadCreate(PIT1Callback,
                                &PIT1Semaphore,
                                &PIT1ThreadStacks[THREAD_STACK_SIZE - 1],
                                1);
        error = OS_ThreadCreate(PacketThread,
                                NULL,
                                &PacketThreadStacks[THREAD_STACK_SIZE - 1],
                                9);
        error = OS_ThreadCreate(RTCThread,
                                RTCSemaphore,
                                &RTCThreadStacks[THREAD_STACK_SIZE - 1],
                                8);
        error = OS_ThreadCreate(TariffDefaults,
                                NULL,
                                &FlashThreadStacks[THREAD_STACK_SIZE - 1],
                                2);

        // Create threads for analog loopback channels
//        for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
//        {
//          error = OS_ThreadCreate(AnalogLoopbackThread,
//                                  &AnalogThreadData[threadNb],
//                                  &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
//                                  ANALOG_THREAD_PRIORITIES[threadNb]);
//        }

        // Start multithreading - never returns!
        OS_Start();
//EnterCritical();
////  DEMInit();
//ExitCritical();


//  for(;;)
//  {
//    if (Packet_Get())
//    {
////      FTM_StartTimer(&aFTMChannel);
////      LEDs_On(LED_BLUE);
//      CommandHandle();
//    }
//  }
}


/*!
 ** @}
 */
