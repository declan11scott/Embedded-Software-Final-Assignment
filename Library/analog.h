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
#define ANALOG_NB_IO  4
#define ANALOG_NB_OUTPUTS 4

// Sine wave array defines
#define SINE_VALUES 6
#define SINE_QUARTER_SAMPLE 5

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;

typedef struct
{
  int16union_t value;                  /*!< The current "processed" analog value (the user updates this value). */
  int16union_t oldValue;               /*!< The previous "processed" analog value (the user updates this value). */
  int16_t values[5];  /*!< An array of sample values to create a "sliding window". */
  int16_t* putPtr;                     /*!< A pointer into the array of the last sample taken. */
} TAnalogInput;

typedef struct AnalogWaveData
{
  int16_t* wavePtr;
  int16_t sine[18];
}TAnalogWaveData;

// Array for Sine wave (2Vpp 0Voffset) over a period.
int16_t sine[18] =
    {
        0xfd09,
        0xf9e5,
        0xf78f,
        0xf642,
        0xf65a,
        0xf795,
        0xf9b2,
        0xfc80,
        0xff9f,
        0x02c6,
        0x05ae,
        0x07fa,
        0x096b,
        0x09d5,
        0x08bb,
        0x0697,
        0x039a,
        0x002a
    };



//      /*! @brief array to identify the direction and polarity of the wave
//       *  @note This can be used for a FSM with sine[]
//       */
//      int wavequarter[4];
//
//      /* @brief FSM set up for wave output
//       * @param value is the pointer to the voltage value
//       * @param quarter is the state of the wave depending on the quadrant it is in
//       */
//      const struct QuarterWaveState
//      {
//        uint16_t *magnitude;
//        int *quarter;
//      };
//      //Define the States
//      #define Q1 &Analog_SineFSM[0]
//      #define Q2 &Analog_SineFSM[1]
//      #define Q3 &Analog_SineFSM[2]
//      #define Q4 &Analog_SineFSM[3]
//
//      // FSM sine wave defined
//      QuarterWaveState Analog_SineFSM[4];

extern TAnalogInput Analog_Input[ANALOG_NB_IO];

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
