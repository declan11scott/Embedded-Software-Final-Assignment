#include "Command.h"


uint16_t Command_16toFloat(float value)
{
  uint16_t i = 100 * value;
  return (uint16_t)i;
}

void Command_Test()
{
  //set DAC with parameters
}

void Command_Tariff(uint8_t tariff)
{
  switch (tariff)
  {
    case 1:
      //load tariff 1
      //Packet_Put(tariff1)
      break;
    case 2:
      //flash read tariff 2
      break;
    case 3:
      //flash read tariff 3
      break;
  }
}

void Command_TimeA()
{
  uint64_t flashTemp;
  uint16union_t data;
  flashTemp = _FP(FLASH_DATA_START);
  data.l = flashTemp & 0xFFFF;
  Packet_Put(0x40,0x00,data.s.Lo,data.s.Hi);
}

void Command_TimeB()
{

}

void Command_Power(uint16union_t power)
{
  Packet_Put(CMD_POWER, power.s.Lo, power.s.Hi, NULL);
}

void Command_Energy(uint32union_t energy)
{
  Packet_Put(CMD_ENERGY, energy.BoP.B[0], energy.BoP.B[1], energy.BoP.B[2]);
}

void Command_Cost(uint16union_t cost)
{
  // calculate cost
  Packet_Put(CMD_COST, cost.s.Lo, cost.s.Hi, NULL);
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

