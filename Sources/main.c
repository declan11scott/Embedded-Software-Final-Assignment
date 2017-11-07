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
#include "HMI.h"

// Baudrates and peripheral constants
#define UARTBaudRate     115200

#define ADCConversion 0xCCC


// Tariff scaled constants by 1000 saved in NvM
#define TARIFF_PEAK       22235
#define TARIFF_SHOULDER   4400
#define TARIFF_OPEAK      2109
#define TARIFF_TWO        1713
#define TARIFF_THREE      4100

#define TARIFF1 1
#define TARIFF2 2
#define TARIFF3 3

#define VOLTAGE_NB	0x00
#define CURRENT_NB  0x01


// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t PIT0ThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PIT1ThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t RTCThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FlashThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t InstantCalcStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PeriodicCalcStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t UARTTxThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t UARTRxThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t HMIThreadStacks[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

uint8_t *NvTariff;
uint16_t Tariff;
float Power[16];
float Energy, iPower, iVoltage, iCurrent;
uint32union_t EnergykWh;
TPowerData	PowerData;

TAnalogOutputData VoltageOutput;
TAnalogOutputData CurrentOutput;

TAnalogInputData VoltageInput;
TAnalogInputData CurrentInput;

uint16_t* InputVoltPtr;
uint16_t* InputCurrPtr;
float* PowerPtr;
uint16union_t PowerW;

uint8_t DisplayOnCnt;

TTariff sTariff;

OS_ECB *FlashSemaphore;
OS_ECB *AnalogGetSemaphore;
OS_ECB *AnalogPutSemaphore;

//-----------------------------------------
// Sine wave set up
//-----------------------------------------

int16_t InstantPower;
uint8_t CurrentMaxCount, VoltageMaxCount[2], TimeDifference, InputFrequency, VoltageCounter, CurrentCounter;
uint16union_t PIT0Frequency;
uint8_t CurrentMaxCtr;
uint8_t VoltageMaxCtr;
float Cost;
float Psi;
int Phase;

static HMIPState StatePtr;

// Flash variables
static volatile uint16union_t* NvTariff1Peak;
static volatile uint16union_t* NvTariff1Shoulder;
static volatile uint16union_t* NvTariff1OPeak;
static volatile uint16union_t* NvTariff2;
static volatile uint16union_t* NvTariff3;

int16_t sine[64] =
    {
		0x0000,	0x0134,	0x0265,	0x0391,	0x04B3,	0x05CB,	0x06D3,	0x07CB,	0x08B0,	0x097F,	0x0A37,	0x0AD6,	0x0B5A,	0x0BC2,	0x0C0D,
		0x0C3A,	0x0C4A,	0x0C3A,	0x0C0D,	0x0BC2,	0x0B5A,	0x0AD6,	0x0A37,	0x097F,	0x08B0,	0x07CB,	0x06D3,	0x05CB,	0x04B3,	0x0391,
		0x0265,	0x0134,	0x0000,	0xFECC,	0xFD9B,	0xFC6F,	0xFB4D,	0xFA35,	0xF92D,	0xF835, 0xF750,	0xF681,	0xF5C9,	0xF52A,	0xF4A6,
		0xF43E,	0xF3F3,	0xF3C6,	0xF3B6,	0xF3C6,	0xF3F3,	0xF43E,	0xF4A6, 0xF52A,	0xF5C9,	0xF681,	0xF750,	0xF835,	0xF92D,	0xFA35,
		0xFB4D,	0xFC6F,	0xFD9B,	0xFECC,
    };
OS_ECB* RTCSemaphore;
OS_ECB* HMISemaphore;

// FSM defined
static HMIFSM HMI_FSM[5] =
{
  {H2, 1},
  {H3, 2},
  {H4, 3},
  {H5, 4},
  {H2, 5}
};

/*! @brief Calculates and outputs the values.
 *
 */
void HMI_Output()
{
  uint8_t days, hours, minutes, seconds;
  uint16_t dollars, integer;
  uint8_t cents, outBuff[256];
  uint8_t* ptrBuff = outBuff;
  uint16_t decimal;

  // Find state and calculate as required
  switch (StatePtr->Output)
  {
    case 2:
      RTC_Get(&hours,&minutes, &seconds);
      days = hours / 24;
      if (days < 99)
        sprintf(outBuff, "\nMetering Time: %02d:%02d:%02d:%02d", days, hours, minutes, seconds);
      else
        sprintf(outBuff, "\nMetering Time: xx:xx:xx:xx");
      break;

    case 3:
      integer = (int)PowerData.Average;
      decimal = (int)(PowerData.Average * 1000) % 1000;
      sprintf(outBuff, "\nAverage Power: %03d.%03d kW", integer, decimal);
      break;

    case 4:
      integer = EnergykWh.s.Lo;
      decimal = EnergykWh.s.Lo % 1000;
      if (integer <= 999)
        sprintf(outBuff, "\nEnergy Consumed: %03d.%03d kWh", integer, decimal);
      else
        sprintf(outBuff, "\nEnergy Consumed: xxx.xxx");
      break;

    case 5:
    dollars = (int)Cost % 100;
    cents = (int)(Cost * 100) % 100;
      if (dollars <= 9999)
        sprintf(outBuff, "\nTotal Cost: $%04d.%02d", dollars, cents);
      else
        sprintf(outBuff, "\nTotal Cost: xxxx.xx");
      break;
  }
  // Print values
  do
  {
    MyUART_OutChar(*ptrBuff);
    ptrBuff++;
  }
  while (*ptrBuff != '\0');
}

/*! @brief Gets correct tariff charge
 *
 *  @return the amount * 1000
 */
uint16_t TariffToUFind()
{
  uint8_t hours;
  RTC_Get(&hours, NULL, NULL);
  if(!(hours % 7))
  {
    if(!(hours % 14))
      // Peak
      Tariff = TARIFF_PEAK;
    else Tariff = TARIFF_SHOULDER;
  }
  if(!(hours % 20))
  {
    Tariff = TARIFF_SHOULDER;
  }
  else Tariff = TARIFF_OPEAK;
  return Tariff;
}

/*! @brief Does necessary calulations for average values every period.
 *
 */
static void PeriodicCalcs()
{
	for(;;)
	{
	  OS_SemaphoreWait(CalculateAvgSemaphore,0);

	  Vrms = 0;
	  Irms = 0;
	  PowerData.Average = 0;

	  for(int i = 0; i < 16; i++)
	  {
      PowerData.Average += PowerData.Instant;
      PowerData.Average /= 16;
      Vrms += Vsqu;
      Irms += Isqu;
      Vrms *= 0.088;
      Irms *= 0.088;
	  }
	  PowerData.PF = PowerData.Average / (Vrms * Irms);
	  // Zero the values that only increment during period.
	  Vsqu = 0;
	  Isqu = 0;
	  PowerData.Instant = 0;
	  VoltageMaxCtr = 0;
	  VoltageMaxCount[0] = 0;
	  VoltageMaxCount[1] = 0;
	  Energy += PowerData.Average;

	  // Calculate cost
    uint16_t aTariff;

    switch(Tariff)
    {
    case TARIFF1:
      aTariff = TariffToUFind();
      break;
    case TARIFF2:
      aTariff = TARIFF_TWO;
      break;
    case TARIFF3:
      aTariff = TARIFF_THREE;
      break;
    default :
      aTariff = TARIFF_OPEAK;
      break;
    }

    //  Calculate_Cost(uint16 TToU, uint16_t Energy
    EnergykWh.l += (int)Energy;
    Cost += ((aTariff * Energy) / 1000000);
    Energy = 0;
	}
}

/*! @brief Does necessary calulations for instantaneous values every sample.
 *
 */
static void InstantCalcs()
{
	for(;;)
	{
		OS_SemaphoreWait(CalculateInstSemaphore,0);
		Vinst = (float)(*VoltageInput.InputPtr) / ADCConversion;
		Iinst = (float)(*CurrentInput.InputPtr) / ADCConversion;
		Vsqu += Vinst * Vinst;
		Isqu += Iinst * Iinst;
		PowerData.Instant += Vinst * Iinst;
		if(Calculate_Largest(*VoltageInput.InputPtr, *(VoltageInput.InputPtr - 1), *(VoltageInput.InputPtr - 2)))
		{
		  if(VoltageMaxCount[0] > 0)
			{
			  VoltageMaxCount[1] = VoltageMaxCtr;
			  VoltageInput.Frequency = (50 * 16) / (2 * (VoltageMaxCount[1] - VoltageMaxCount[0])) ;
			}
		  else VoltageMaxCount[0] = VoltageMaxCtr;
		}
		else
		{
		  VoltageInput.LargestCount[0]++;
		}
		VoltageMaxCtr++;
	}
}

/*! @brief Sets correct initial tariff value to the global variable.
 *
 *  @note Assumes that Flash_Init has been called.
 */
void TariffDefaults()
{
  bool success;
  // Allocate memory in flash will return true if successful
  success = Flash_AllocateVar((volatile void**)&NvTariff, sizeof(*NvTariff));

  // Check if there's no previously set value in NvTowerNb in flash memory
  if (success && *NvTariff == 0xFFFF)
  {
    // If it's empty, just allocate it the default value - Tariff 1
    success &= Flash_Write8((uint8_t*)NvTariff, TARIFF1);
  }

  // Initialise flash just because
  uint64_t initialiseFlash = _FP(FLASH_DATA_START);

  OS_ThreadDelete(OS_PRIORITY_SELF);
}

//-----------------------Peripheral Threads--------------------------------------
/*! @brief Rx Thread for UART receive
 *
 */
static void TxThread(void* arg)
{
  for(;;)
  {
   OS_SemaphoreWait(UARTTxSemaphore,0);
   MyFIFO_Get(&TX_FIFO, &UART2_D);
   UART2_C2 |= UART_C2_TIE_MASK;
  }
}

/*! @brief Rx Thread for UART receive
 *
 */
static void RxThread(void* arg)
{
  for(;;)
  {
   OS_SemaphoreWait(UARTRxSemaphore,0);
   MyFIFO_Put(&RX_FIFO, MyUART_TempRxData);
  }
}

/*! @brief Thread routine for the RTC.
 *         Updates HMI if necessary
 */
static void RTCThread(void* args)
{
  for(;;)
  {
    OS_SemaphoreWait(RTCSemaphore,0);

    // Display HMI if the count has been updated
    if(DisplayOnCnt > 0)
    {
      // If state is not dormant, update and decrement count
  	  if(StatePtr != H1)
  	  {
  		  HMI_Output();
  		  DisplayOnCnt--;
  	  }
  	  else StatePtr = H1;
    }
  }
}
//-------------------------------------------------------------------------------

//--------------------------Timer Callbacks----------------------------------------
/*! @brief Callback function for the RTC. Signals for HMI update
 *
 *  @note Assumes RTC has been initialized.
 */
static void RTCCallback(void *args)
{
  OS_ISREnter();

  OS_SemaphoreSignal(RTCSemaphore);

  OS_ISRExit();
}

/*! @brief Callback function for the PIT. Signals for calculations
 *
 *  @note Assumes PIT has been initialized.
 */
static void PIT0Callback(void* args)
{
  for(;;)
  {
    OS_SemaphoreWait(PIT0Semaphore, 0);

    // Signal for instant calculations to occur
    OS_SemaphoreSignal(CalculateInstSemaphore);

    // Check the input pointer and loopback if at the end
    if(VoltageInput.InputPtr == &VoltageInput.InputValues[15])
    {
      VoltageInput.InputPtr = VoltageInput.InputValues;
      CurrentInput.InputPtr = CurrentInput.InputValues;
      OS_SemaphoreSignal(CalculateAvgSemaphore);
    }
    else
    {
      VoltageInput.InputPtr++;
      CurrentInput.InputPtr++;
    }
    // Check the output pointer and loopback if at the end
    if (VoltageOutput.wavePtr > &sine[59])
    {
      VoltageOutput.wavePtr = sine;
    }
    else
    {
      VoltageOutput.wavePtr+=4;
    }

    if (CurrentOutput.wavePtr > &sine[59])
    {
      CurrentOutput.wavePtr = sine;
    }
    else
    {
      CurrentOutput.wavePtr+=4;
    }
  }
}
//---------------------------------------------------------------------------------------

//--------------------------Peripheral ISR's----------------------------------------
/*! @brief Interrupt service routine for the PIT.
 *
 *  @note Assumes initialisation, set and enable have occurred
 */
void __attribute__ ((interrupt)) PIT0_ISR(void)
{
  OS_ISREnter();

  // Clear flag and signal semaphore
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

  // Sample data on both channels
  Analog_Get(VOLTAGE_NB, VoltageInput.InputPtr);
  Analog_Get(CURRENT_NB, CurrentInput.InputPtr);
  // Output if pointing at array
  Analog_Put(VOLTAGE_NB, *VoltageOutput.wavePtr);
  Analog_Put(CURRENT_NB, *CurrentOutput.wavePtr);

  (void)OS_SemaphoreSignal(PIT0Semaphore);

  OS_ISRExit();
}

/*! @brief Interrupt service routine for the HMI button SW1.
 *
 *  @note Assumes initialisation hase occurred
 */
void __attribute__ ((interrupt)) HMI_ISR(void)
{
  OS_ISREnter();
  // w1c flag
  PORTD_PCR0 |= PORT_PCR_ISF_MASK;
  // signal semaphore and set count to decremenent by RTC over 15s
  DisplayOnCnt = 15;
  // Push state out of the dormant state
  StatePtr = StatePtr->NextState;
  OS_ISRExit();
}
//------------------------------------------------------------------------

/*! @brief Sets test wave to zero.
 *
 */
static void ZeroAnalog()
{
	while (PowerData.Average > 10)
	{
    Analog_Put(0x00,0);
    Analog_Put(0x01,0);
    VoltageOutput.wavePtr = 0;
    CurrentOutput.wavePtr = 0;
 	}
}

/*! @brief Initialises test sine wave.
 *
 */
void TestMode()
{
  if(Packet_Parameter1 == 0x01)
  {
	  VoltageOutput.wavePtr = sine;
	  CurrentOutput.wavePtr = sine + Phase;
  }
  else
  {
    ZeroAnalog();
  }
}

/*! @brief Initialises peripherals.
 *
 */
static void DEMInit()
{
  // Initialise peripherals
  MyPacket_Init(UARTBaudRate, CPU_BUS_CLK_HZ);
  Flash_Init();
  MyPIT_Init(PIT0Callback,NULL,NULL,NULL, NULL);
  RTC_Init(RTCCallback, NULL);
  Analog_Init(CPU_BUS_CLK_HZ);
  HMI_Init();

  // Pointers
  // Initialise output sine wave pointers for test mode
  VoltageOutput.wavePtr = 0;
  CurrentOutput.wavePtr = 0;

  // Input values pointing to start of array
  VoltageInput.InputPtr = VoltageInput.InputValues;
  CurrentInput.InputPtr = CurrentInput.InputValues;

  // HMI initial dormant state
  StatePtr = H1;

  // Self test default frequency
  PIT0Frequency.l = 50;

  // Set and enable the sampling PIT
  MyPIT_Set((uint32_t)PIT0Frequency.l, PIT_SELECT_0); // assign the period
  MyPIT_Enable(true, PIT_SELECT_0);
}

/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{
  // Initialise peripherals for Digital Energy Meter
  DEMInit();

  // Create semaphores
  RTCSemaphore        	 = OS_SemaphoreCreate(0);
  PIT0Semaphore       	 = OS_SemaphoreCreate(0);
  PIT1Semaphore          = OS_SemaphoreCreate(0);
  CalculateInstSemaphore = OS_SemaphoreCreate(0);
  CalculateAvgSemaphore  = OS_SemaphoreCreate(0);
  PacketSemaphore		     = OS_SemaphoreCreate(1);

  // Make sure output is zero
  ZeroAnalog();

  // Delete after everything has been initialised
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Handles the response to a packet sent from PC
 *  @note Assumes MyPacketInit() has been called
 */
void CommandHandle()
{
  uint8_t dCost, cCost, hours, seconds, minutes, aTariff;
  uint16union_t Freqx10, rVoltage, rCurrent, iPower, PF;
  int16_t phase;

  switch (Packet_Command)
    {
      case CMD_TEST:
        TestMode();
        break;

      case CMD_TARIFF:
    	  Tariff = Packet_Parameter1;
	   	  Flash_Write8((uint8_t*)NvTariff, Packet_Parameter1);
        break;

      case CMD_TIME_A:
        RTC_Get(&hours, &minutes, &seconds);
        Command_TimeA(seconds, minutes);
        break;

      case CMD_TIME_B:
        RTC_Get(&hours, &minutes, &seconds);
        Command_TimeB(minutes);
        break;

      case CMD_POWER:
        iPower.l = 100 * PowerData.Average;
    	  Command_Power(iPower);
        break;

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

      case CMD_PF:
    	  PF.l = (uint16_t)(1000 * PowerData.PF);
        Command_PowerFactor(PF); //create this command
        break;

      case CMD_FREQ_ENTER:
        PIT0Frequency.l = Packet_Parameter12 / 10;
        MyPIT_Enable(false, PIT_SELECT_0);
        MyPIT_Set((uint32_t)PIT0Frequency.l, PIT_SELECT_0); // assign the period
        MyPIT_Enable(true, PIT_SELECT_0);
        break;

//      case CMD_VOLTAGE_ENTER:
//        // Voltage * 10
//
//        VoltageOutput.RMS.l = (int16_t)Packet_Parameter12;
//      break;
//
//      case CMD_CURRENT_ENTER:
//        // Current * 10
//        CurrentOutput.RMS.l = (int16_t)Packet_Parameter12;
//      break;
//
      case CMD_PHASE_ENTER:
        phase = Packet_Parameter12 / 5625;
        if(phase < 0)
    	    Phase = 64 - phase;
        else Phase = phase;
        break;

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
        error = OS_ThreadCreate(PacketThread,
                                NULL,
                                &PacketThreadStacks[THREAD_STACK_SIZE - 1],
                                8);
        error = OS_ThreadCreate(RTCThread,
                                RTCSemaphore,
                                &RTCThreadStacks[THREAD_STACK_SIZE - 1],
                                6);
        error = OS_ThreadCreate(TariffDefaults,
                                NULL,
                                &FlashThreadStacks[THREAD_STACK_SIZE - 1],
                                1);
        error = OS_ThreadCreate(InstantCalcs,
                                CalculateInstSemaphore,
                                &InstantCalcStacks[THREAD_STACK_SIZE - 1],
                                5);
        error = OS_ThreadCreate(PeriodicCalcs,
                                CalculateAvgSemaphore,
                                &PeriodicCalcStacks[THREAD_STACK_SIZE - 1],
                                7);
        error = OS_ThreadCreate(RxThread,
                                &UARTRxSemaphore,
                                &UARTRxThreadStacks[THREAD_STACK_SIZE - 1],
                                3);
        error = OS_ThreadCreate(TxThread,
                                &UARTTxSemaphore,
                                &UARTTxThreadStacks[THREAD_STACK_SIZE - 1],
                                2);
//        error = OS_ThreadCreate(HMI_Output,
//                                &HMISemaphore,
//                                &HMIThreadStacks[THREAD_STACK_SIZE - 1],
//                                2);



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
