/*! @file
 *
 *  @brief Routines for setting up and reading from the ADC.
 *
 *  This contains the functions for reading analog values from the LTC1859 ADC on the TWR-ADCDAC-LTC board.
 *  The ADC is 16-bit, and configured with a bipolar voltage range of +/- 10V.
 *  This contains the functions for writing analog values to the LTC2704 DAC on the TWR-ADCDAC-LTC board.
 *  The DAC is 16-bit, and configured with a bipolar voltage range of +/- 10V.
 *
 *  @author PMcL
 *  @date 2016-09-23
 */

#ifndef ANALOG_H
#define ANALOG_H

// new types
#include "types.h"

// Maximum number of channels
#define ANALOG_NB_IO  2
#define ANALOG_NB_OUTPUTS 4

// Sine wave array defines
#define SINE_VALUES 6
#define SINE_QUARTER_SAMPLE 5

#define Vrms  VoltageInput.RMS
#define Irms  CurrentInput.RMS
#define Vsqu  VoltageInput.InstantSquared
#define Isqu  CurrentInput.InstantSquared
#define Vinst VoltageInput.Instant
#define Iinst CurrentInput.Instant

typedef struct AnalogWaveData
{
  int16_t* wavePtr;
  int16union_t RMS;
  uint16union_t Frequency;
  uint16union_t Phase;
}TAnalogOutputData;

typedef struct InputPowerData
{
  float Average;
  float Instant;
  float PF;
}TPowerData;

typedef struct AnalogInputData
{
  int16_t InputValues[16];
  int16_t* InputPtr;
  float RMS;
  float Phase;
  float Frequency;
  uint16_t Power;
  uint8_t LargestCount[2];
  float Instant;
  float InstantSquared;
  float PF;
}TAnalogInputData;

extern TAnalogInputData VoltageInput;
extern TAnalogInputData CurrentInput;
extern TPowerData		PowerData;

/*! @brief Sets up the ADC before first use.
 *
 *  @param moduleClock The module clock rate in Hz.
 *  @return bool - true if the UART was successfully initialized.
 */
bool Analog_Init(const uint32_t moduleClock);

/*! @brief Gets a value from an analog input channel.
 *
 *  @param channelNb is the number of the analog input channel to get a value from.
 *  @param valuePtr A pointer to a memory location to place the analog value.
 *  @return bool - true if the analog value was acquired successfully.
 */
bool Analog_Get(const uint8_t channelNb, const uint16_t* data);

/*! @brief Sends a value to an analog input channel.
 *
 *  @param channelNb is the number of the analog output channel to send the value to.
 *  @param value is the value to write to the analog channel.
 *  @return bool - true if the analog value was output successfully.
 */
bool Analog_Put(uint8_t const channelNb, int16_t const value);

#endif
