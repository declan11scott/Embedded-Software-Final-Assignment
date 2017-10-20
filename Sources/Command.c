#include "Command.h"

void Command_Test(uint16_t voltage, uint16_t current, uint16_t phase)
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

}

void Command_TimeB()
{

}

void Command_Power()
{

}

void Command_Energy()
{

}

void Command_Cost()
{
  // calculate cost
}

void Command_Frequency()
{

}

void Command_Voltage()
{

}

void Command_Current()
{

}

void Command_PowerFactor()
{

}

