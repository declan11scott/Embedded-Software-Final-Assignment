/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author PMcL
 *  @date 2015-08-22
 */

#ifndef PIT_H
#define PIT_H

// new types
#include "types.h"
#include "OS.h"
#include "analog.h"

#define PIT_SELECT_0 1
#define PIT_SELECT_1 2
#define PIT_SELECT_2 4
#define PIT_SELECT_3 8

extern OS_ECB *PIT0Semaphore;
extern OS_ECB *PIT1Semaphore;


/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
void MyPIT_Init();

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void MyPIT_Set(const uint32_t period, const uint8_t pitNb);

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void MyPIT_Enable(const bool enable, uint8_t pitNb);

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT0_ISR(void);
//void __attribute__ ((interrupt)) PIT1_ISR(void);
//void __attribute__ ((interrupt)) PIT2_ISR(void);
//void __attribute__ ((interrupt)) PIT3_ISR(void);

#endif
