/*
 * packet.c
 *
 *  Created on: 1 Aug 2017
 *  @date: 8 Aug 2017
 *  @author: 11970744, 11986282
 */
/*!
**  @addtogroup packet_module packet module documentation
**  @{
*/
/* MODULE packet */

#include "packet.h"
#include "MyUART.h"

TPacket Packet;
static uint8_t PacketPosition;

OS_ECB* PacketSemaphore;


/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 *  @bool Checksum_Check Compares the calculated checksum with the value of Checksum and returns true if the are the same
 */
static bool Checksum_Check(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
   uint8_t Calculated_Checksum = command ^ parameter1 ^ parameter2 ^ parameter3;
   if (Calculated_Checksum == Packet_Checksum)
   {
      return true;
   }
   return false;
}

bool MyPacket_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
   PacketPosition = 0;
   Packet_Parameter1 = 0;
   Packet_Parameter2 = 0;
   Packet_Parameter3 = 0;
   Packet_Checksum = 0;
   Packet_Command = 0;

   return MyUART_Init(baudRate, moduleClk);
}

void Packet_Shift()
{
   Packet_Command = Packet_Parameter1;
   Packet_Parameter1 = Packet_Parameter2;
   Packet_Parameter2 = Packet_Parameter3;
   Packet_Parameter3 =  Packet_Checksum;
   Packet_Checksum = 0;
}

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool MyPacket_Get(void)
{
   static int PacketPosition = 0;

   switch(PacketPosition){
	 case 0:
	   if (MyUART_InChar(&Packet_Command))
	     PacketPosition++;
	  break;

	case 1:
	  if (MyUART_InChar(&Packet_Parameter1))
	    PacketPosition++;
	  break;

	case 2:
	  if (MyUART_InChar(&Packet_Parameter2))
		PacketPosition++;
	  break;

	case 3:
	  if (MyUART_InChar(&Packet_Parameter3))
	    PacketPosition++;
	  break;

	case 4:
	  if (MyUART_InChar(&Packet_Checksum))
		PacketPosition++;
	  break;

	case 5:
	  if (Checksum_Check(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3))
	  {
	     PacketPosition = 0;
		 return true;
	  }
	  else
	  {
	     PacketPosition = 4;
		 Packet_Shift();
	  }
	  break;

	default:
	    PacketPosition = 0;
	  break;
	}

	return false;
}

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *  call 5 times - shift if the check sum is not true
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool MyPacket_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
   OS_SemaphoreWait(PacketSemaphore,0);
   uint8_t checkSum = command ^ parameter1 ^ parameter2 ^ parameter3;
   /* call UART_OutChar() 5 times 5th byte is calculated -> Checksum */
   bool success = MyUART_OutChar(command)
		   && MyUART_OutChar(parameter1)
		   && MyUART_OutChar(parameter2)
		   && MyUART_OutChar(parameter3)
		   && MyUART_OutChar(checkSum);

   OS_SemaphoreSignal(PacketSemaphore);

   return success;
}

//call UART_outchar 5 times. 5th time is check sum

/* END packet */
/*!
** @}
*/
