/*
 * MyPIT.c
 *
 *  Created on: Oct 28, 2017
 *      Author: Declan
 */
/*! @file PIT.c
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Gideon Kanikevich
 *  @date 2017-06-27
 */

// new types
#include "types.h"
#include "PIT.h"
#include "MK70F12.h"
#include "CPU.h"

static void (*Callback0)(void *);
static void (*Callback1)(void *);
static void (*Callback2)(void *);
static void (*Callback3)(void *);
static void *Arguments;

//    OS_ECB *PIT1Semaphore;
//    OS_ECB *PIT2Semaphore;
//    OS_ECB *PIT3Semaphore;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
void MyPIT_Init(void (*userFunction0)(void*),
                void (*userFunction1)(void*),
                void (*userFunction2)(void*),
                void (*userFunction3)(void*), void* userArguments)
{
  // For all PITs
  // system clock control gate control gate system control clock
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
  // enter the fro-zone if you're in the debug-zone
  // also disables MDIS
  PIT_MCR = PIT_MCR_FRZ_MASK;

  // For PIT0
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
  // enable interrupts
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

  //NVIC info n stuff
  // vector 84 = 0x54, IRQ = 68, non-IPR = 2, IPR = 17
  // 68 % 32 = 4
  NVICISER2 |= NVIC_ISER_SETENA(1 << 4);
  NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << 4);

  // For PIT1
  PIT_TFLG1 |= PIT_TFLG_TIF_MASK;
  // enable interrupts
  PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;

  //NVIC info n stuff
  // vector 85 = 0x55, IRQ = 69, non-IPR = 2, IPR = 17
  // 69 % 32 = 5
  NVICISER2 |= NVIC_ISER_SETENA(1 << 5);
  NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << 5);

  // For PIT2
  PIT_TFLG2 |= PIT_TFLG_TIF_MASK;
  // enable interrupts
  PIT_TCTRL2 |= PIT_TCTRL_TIE_MASK;

  //NVIC info n stuff
  // vector 86 = 0x56, IRQ = 70, non-IPR = 2, IPR = 17
  // 70 % 32 = 6
  NVICISER2 |= NVIC_ISER_SETENA(1 << 6);
  NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << 6);

  // For PIT3
  PIT_TFLG3 |= PIT_TFLG_TIF_MASK;
  // enable interrupts
  PIT_TCTRL3 |= PIT_TCTRL_TIE_MASK;

  //NVIC
  // vector 87 = 0x57, IRQ = 71, non-IPR = 2, IPR = 17
  // 71 % 32 = 7
  NVICISER2 |= NVIC_ISER_SETENA(1 << 7);
  NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << 7);

  // User Functions
  Callback0 = userFunction0;
  Callback1 = userFunction1;
  Callback2 = userFunction2;
  Callback3 = userFunction3;
  Arguments = userArguments;

}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void MyPIT_Set(const uint32_t freq, const uint8_t pitNb)
{
//  uint32_t freq = 1000000000/period;
  uint32_t cycles = CPU_BUS_CLK_HZ/(16 * freq) - 1;
  switch(pitNb)
  {
    case PIT_SELECT_0:
    PIT_LDVAL0 |= PIT_LDVAL_TSV(cycles);
    break;

    case PIT_SELECT_1:
    PIT_LDVAL1 |= PIT_LDVAL_TSV(cycles);
    break;

    case PIT_SELECT_2:
    PIT_LDVAL2 |= PIT_LDVAL_TSV(cycles);
    break;

    case PIT_SELECT_3:
    PIT_LDVAL3 |= PIT_LDVAL_TSV(cycles);
    break;

  }
}
/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void MyPIT_Enable(const bool enable, uint8_t pitNb)
{
  switch (pitNb)
  {
    case PIT_SELECT_0:
      if (enable)
      {
        PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
      }
      else
      {
        PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
      }
      break;

    case PIT_SELECT_1:
      if (enable)
      {
        PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;
      }
      else
      {
        PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK;
      }
      break;

    case PIT_SELECT_2:
      if (enable)
      {
        PIT_TCTRL2 |= PIT_TCTRL_TEN_MASK;
      }
      else
      {
        PIT_TCTRL2 &= ~PIT_TCTRL_TEN_MASK;
      }
      break;

    case PIT_SELECT_3:
      if (enable)
      {
        PIT_TCTRL3 |= PIT_TCTRL_TEN_MASK;
      }
      else
      {
        PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;
      }
      break;
  }

}

void __attribute__ ((interrupt)) PIT0_ISR(void)
{
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
  (*Callback0)(Arguments);
}
void __attribute__ ((interrupt)) PIT1_ISR(void)
{
  PIT_TFLG1 |= PIT_TFLG_TIF_MASK;
  (*Callback1)(Arguments);
}
void __attribute__ ((interrupt)) PIT2_ISR(void)
{
  PIT_TFLG2 |= PIT_TFLG_TIF_MASK;
  (*Callback2)(Arguments);
}
void __attribute__ ((interrupt)) PIT3_ISR(void)
{
  PIT_TFLG3 |= PIT_TFLG_TIF_MASK;
  (*Callback3)(Arguments);
}

