/*! @file
 *
 *  @brief I/O routines for K70 SPI interface.
 *
 *  This contains the functions for operating the SPI (serial peripheral interface) module.
 *
 *  @author Declan Scott         11970744
 *  @author Christine Vinaviles  11986282
 *  @date 2017-09-26
 */
#include "SPI.h"
#include "MK70F12.h"
#include "stdlib.h"

static uint8_t pbrValues[4] = {2, 3, 5, 7};
static uint32_t brValues[16] = {2, 4, 6, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};

static uint8_t pdtValues[4] = {1, 3, 5, 7};
static uint32_t dtValues[16] = {2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

/*! @brief returns absolute value.
 *
 *  @param nb is the variable to make sure is positive.
 *
 *  @return float
 */
static float Abs(float nb)
{
  if (nb < 0)
  {
    return nb * -1;
  }
  else
  {
    return nb;
  }
}

/*! @brief finds ideal baud rate and delay values for CTAR register.
 *
 *  @param IdealNb is the ideal baud rate or delay value to calcuate.
 *  @param moduleClk The module clock in Hz.
 *  @param ptf_array is the pointer to the smaller array [4] (br or dt)
 *  @param ptos_array is the pointer to the larger array [16] (pbr or pdt)
 *
 *  @notes assumes the pre-scalar is zero.
 */
  static void Idealbaudanddelayfinder(uint32_t IdealNb, uint32_t moduleClock, uint8_t* ptf_array, uint8_t* ptos_array, BaudDelay BD)
  {
    uint32_t sixteenarray[16] ;
    uint8_t fourarray[4];
    float ideal;
    //baudrate or delay? Set up arrays for either case.
    switch(BD)
    {
      case Baudrate:
        for (int a = 0; a < 16; a++)
        {
          sixteenarray[a] = brValues[a];
        }

        for (int a = 0; a < 16; a++)
        {
          fourarray[a] = pbrValues[a];
        }
        //ideal ratio is inverse for the delay as it is calculated with time not freq.
        ideal = (float)IdealNb / (float)moduleClock;
        break;
      case Delay:
        for (int a = 0; a < 16; a++)
        {
          sixteenarray[a] = dtValues[a];
        }

        for (int a = 0; a < 16; a++)
        {
          fourarray[a] = pdtValues[a];
        }
        ideal = (float)moduleClock / (float)IdealNb;
        break;
    }

    float bestResult = 0, tmpResult;
    uint8_t i, j, pt_four, pt_sixteen;

    for (i = 0; i < 4; i++)
    {
      for (j = 0; j < 16; j++)
      {
        tmpResult = 1 / ((float)(fourarray[i] * sixteenarray[j]));

        if (Abs(ideal - tmpResult) < Abs(ideal - bestResult) || bestResult == 0)
        {
          bestResult = tmpResult;
          pt_four = i;
          pt_sixteen = j;
        }
      }
    }

    *ptf_array = pt_four;
    *ptos_array =  pt_sixteen;

  }

bool SPI_Init(const TSPIModule* const aSPIModule, const uint32_t moduleClock)
{
  /* Enable SPI2, PORTD and PORTE clock gate control */
  SIM_SCGC3 |= SIM_SCGC3_DSPI2_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  /* Enable SPI2 signals */
  PORTD_PCR11 |= PORT_PCR_MUX(0x02);
  PORTD_PCR12 |= PORT_PCR_MUX(0x02);
  PORTD_PCR13 |= PORT_PCR_MUX(0x02);
  PORTD_PCR14 |= PORT_PCR_MUX(0x02);
  PORTD_PCR15 |= PORT_PCR_MUX(0x02);

  /* Set up PTE5 & PTE27 as outputs */
  PORTE_PCR5 |= PORT_PCR_MUX(0x01);
  PORTE_PCR27 |= PORT_PCR_MUX(0x01);

  /* Sets pins as outputs */
  GPIOE_PDDR |= (1<<5);
  GPIOE_PDDR |= (1<<27);

  /* Set EOQF flag to clear TXRXS bit */
  SPI2_SR |= SPI_SR_EOQF_MASK;

  /* Clear transmit flag */
  SPI2_SR |= SPI_SR_TFFF_MASK;

  //SPI2 MCR
  SPI2_MCR    |= SPI_MCR_HALT_MASK;
  //SPI2_SR     |= SPI_SR_EOQF_MASK; // tx and rx
  SPI2_MCR    |= aSPIModule->isMaster << SPI_MCR_MSTR_SHIFT;
  SPI2_MCR    |= aSPIModule->continuousClock << SPI_MCR_CONT_SCKE_SHIFT;
  SPI2_MCR    |= SPI_MCR_FRZ_MASK;
  SPI2_MCR    |= SPI_MCR_PCSIS(1);
  SPI2_MCR    &= ~SPI_MCR_MDIS_MASK;
  SPI2_MCR    |= SPI_MCR_DIS_RXF_MASK;
  SPI2_MCR    |= SPI_MCR_DIS_TXF_MASK;

  //SPI2 CTAR0
  SPI2_CTAR0  |= SPI_CTAR_FMSZ(15);
  SPI2_CTAR0  |= aSPIModule->LSBFirst << SPI_CTAR_LSBFE_SHIFT;
  SPI2_CTAR0  |= aSPIModule->changedOnLeadingClockEdge << SPI_CTAR_CPHA_SHIFT;
  SPI2_CTAR0  |= aSPIModule->inactiveHighClock << SPI_CTAR_CPOL_SHIFT;

  //BAUDRATE
  uint8_t pbr, br;
  BaudDelay BD = Baudrate;
  //set the values for PBR and BR

  //search through all values to find closest match.
  Idealbaudanddelayfinder(aSPIModule->baudRate, moduleClock, &pbr, &br, BD);

  //put values into the registers
  SPI2_CTAR0 &= ~SPI_CTAR_DBR_MASK;
  SPI2_CTAR0 |= SPI_CTAR_PBR(pbr);
  SPI2_CTAR0 |= SPI_CTAR_BR(br);

  //delay after transmit to be 5us
  //similar to baud rate calc.
  uint8_t pdt, dt;
  BD = Delay;
  uint32_t delayfreqinv = 200000;
  Idealbaudanddelayfinder(delayfreqinv, moduleClock, &pdt, &dt, BD);
  SPI2_CTAR0 |= SPI_CTAR_DT(dt);
  SPI2_CTAR0 |= SPI_CTAR_PDT(pdt);

  //start SPI
  SPI2_MCR    &= ~SPI_MCR_HALT_MASK;
  return true;

}

void SPI_SelectSlaveDevice(const uint8_t slaveAddress)
{
  //use the GPIO to select the Slave device
  //GPIO 7 = 8 = 9 = high for ADC
  //GPIO 7 = 8 = high. GPIO 9 = low for DAC
  //gpio 7 = PTE27
  //gpio8 = pte5
  //gpio9 = pte18 (always high

  switch (slaveAddress)
  {
    //case 4/5/6/7:

    case 0x07: //ADC
      GPIOE_PDOR |= (1 << 5);
      GPIOE_PDOR |= (1 << 27);
      break;
    case 0x01: //DAC
      GPIOE_PDOR |= (0 << 5);
      GPIOE_PDOR |= (0 << 27);
      break;
  }

  return;
}

void SPI_Exchange(const uint16_t dataTx, uint16_t* const dataRx)
{

  //WAIT FOR TFFF THEN CLEAR
  while (!(SPI2_SR & SPI_SR_TFFF_MASK))
  {

  }


  SPI2_SR |= SPI_SR_TFFF_MASK;
  //clear pushr
  //SPI2_PUSHR &= SPI_PUSHR_TXDATA(00);
  //set PUSHR commands
  SPI2_PUSHR |= SPI_PUSHR_PCS(1);
  SPI2_PUSHR |= SPI_PUSHR_TXDATA(dataTx);

  for(int i = 0; i < 100; i++);

  while (!(SPI2_SR & SPI_SR_RFDF_MASK))
  {

  }

  if(dataRx){
    *dataRx = SPI2_POPR;
  } else {
    SPI2_POPR;
  }

  //clear flags
  SPI2_SR |= SPI_SR_RFDF_MASK;


  //clear FIFO's
//  SPI2_MCR |= SPI_MCR_CLR_RXF_MASK;
//  SPI2_MCR |= SPI_MCR_CLR_TXF_MASK;

}
