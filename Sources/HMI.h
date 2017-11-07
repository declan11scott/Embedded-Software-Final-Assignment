/*
 * HMI.h
 *
 *  Created on: 6 Nov 2017
 *      Author: 11970744
 */

#ifndef SOURCES_HMI_H_
#define SOURCES_HMI_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "OS.h"

//Define the States
#define H1 &HMI_FSM[0]
#define H2 &HMI_FSM[1]
#define H3 &HMI_FSM[2]
#define H4 &HMI_FSM[3]
#define H5 &HMI_FSM[4]

typedef const struct HFSM
{
//  uint8_t *State;
  const struct HFSM  *NextState;
  uint8_t Output;
}HMIFSM;

typedef HMIFSM *HMIPState;

extern OS_ECB* HMISemaphore;



void HMI_Init();

void __attribute__ ((interrupt)) HMI_ISR(void);

#endif /* SOURCES_HMI_H_ */
