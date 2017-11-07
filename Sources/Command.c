#include "Command.h"
#include "Flash.h"

uint16_t Command_16toFloat(float value)
{
  uint16_t i = 100 * value;
  return (uint16_t)i;
}

void Command_Test()
{
    uint64_t flashTemp;
    uint32union_t data1,data2;
    flashTemp = _FP(FLASH_DATA_START);
    data1.l = flashTemp & 0xFFFFFFFF;
    data2.l = (flashTemp >> 32) & 0xFFFFFFFF;
    Packet_Put(data1.BoP.B[0],data1.BoP.B[1],data1.BoP.B[2],data1.BoP.B[3]);
    Packet_Put(data2.BoP.B[0],data2.BoP.B[1],data2.BoP.B[2],data2.BoP.B[3]);

}

uint8_t Command_Tariff(uint8_t tariff)
{
  switch (tariff)
  {
    case 1:

      break;
    case 2:
      //flash read tariff 2
      break;
    case 3:
      //flash read tariff 3
      break;
  }
}

void Command_TimeA(uint8_t seconds, uint8_t minutes)
{
  Packet_Put(CMD_TIME_A, seconds, minutes, 0);
}

void Command_TimeB(uint8_t minutes)
{
  uint8_t hours = minutes / 60;
  uint8_t days  = hours / 24;
  Packet_Put(CMD_TIME_B, hours, days, 0);
}

void Command_Power(uint16union_t power)
{
  Packet_Put(CMD_POWER, power.s.Lo, power.s.Hi, NULL);
}

void Command_Energy(uint32union_t energy)
{
  Packet_Put(CMD_ENERGY, energy.BoP.B[0], energy.BoP.B[1], energy.BoP.B[2]);
}

void Command_Cost(uint8_t cents, uint8_t dollarydoos)
{
  // calculate cost
  Packet_Put(CMD_COST, cents, dollarydoos, NULL);
}

void Command_Frequency(uint16union_t frequency)
{
  Packet_Put(CMD_FREQUENCY, frequency.s.Lo, frequency.s.Hi, NULL);
}

void Command_Voltage(uint16union_t voltage)
{
  Packet_Put(CMD_VOLTAGE, voltage.s.Lo, voltage.s.Hi, NULL);
}

void Command_Current(uint16union_t current)
{
  Packet_Put(CMD_CURRENT, current.s.Lo, current.s.Hi, NULL);
}

void Command_PowerFactor(uint16union_t pf)
{
  Packet_Put(CMD_PF, pf.s.Lo, pf.s.Hi, NULL);
}

