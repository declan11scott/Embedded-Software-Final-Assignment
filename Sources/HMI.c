/*
 * HMI.c
 *
 *  Created on: 6 Nov 2017
 *      Author: 11970744
 */
#include "MK70F12.h"
#include "HMI.h"
#include "PE_Types.h"
#include "RTC.h"
#include "MyUART.h"

#define BUFFER_LENGTH 256

void HMI_Init()
{
	//Initialise Port D
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD_PCR0 |= PORT_PCR_MUX(1);
	PORTD_PCR0 |= PORT_PCR_IRQC(8);
	PORTD_PCR0 |= PORT_PCR_PE_MASK;
	PORTD_PCR0 |= PORT_PCR_PS_MASK;

	NVICISER2 |= NVIC_ISER_SETENA(1 << (90 % 32));
	NVICICPR2 |= NVIC_ICPR_CLRPEND(1 << (90 % 32));

}


