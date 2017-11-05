/*
 * Tariff.h
 *
 *  Created on: 5 Nov 2017
 *      Author: 11970744
 */

#ifndef SOURCES_TARIFF_H_
#define SOURCES_TARIFF_H_

#include "Flash.h"

void Tariff_Save(volatile uint16union_t* tariff1Peak, volatile uint16union_t* tariff1Shoulder, volatile uint16union_t* tariff1OPeak, volatile uint16union_t* tariff2, volatile uint16union_t* tariff3);

#endif /* SOURCES_TARIFF_H_ */
