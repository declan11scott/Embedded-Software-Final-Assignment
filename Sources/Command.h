
#include "Flash.h"
#include "packet.h"
#include "PE_Types.h"
#include "types.h"


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
#define CMD_PF  		  0x1A

#define CMD_FREQ_ENTER    0x27
#define CMD_VOLTAGE_ENTER 0x28
#define CMD_CURRENT_ENTER 0x29
#define CMD_PHASE_ENTER      0x30

//typedef struct
//{
//  uint16union_t power;
//  uint32union_t energy;
//  uint16union_t cost;
//  uint16union_t frequency;
//  uint16union_t voltage;
//  uint16union_t current;
//  uint16union_t pf;
//} CMDStruct;



void Command_Test();

uint8_t Command_Tariff(uint8_t tariff);

void Command_TimeA(uint8_t seconds, uint8_t minutes);

void Command_TimeB(uint8_t minutes);

void Command_Power(uint16union_t power);

void Command_Energy(uint32union_t energy);

void Command_Cost(uint8_t cents, uint8_t dollarydoos);

void Command_Frequency(uint16union_t frequency);

void Command_Voltage(uint16union_t voltage);

void Command_Current(uint16union_t current);

void Command_PowerFactor(uint16union_t pf);

