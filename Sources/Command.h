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
#define CMD_POWER_FACTOR  0x20

#include "types.h"

void Command_Test(uint16_t voltage, uint16_t current, uint16_t phase);

void Command_Tariff(uint8_t tariff);

void Command_TimeA();

void Command_TimeB();

void Command_Power();

void Command_Energy();

void Command_Cost();

void Command_Frequency();

void Command_Voltage();

void Command_Current();

void Command_PowerFactor();

