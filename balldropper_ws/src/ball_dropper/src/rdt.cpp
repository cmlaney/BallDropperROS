/*
 * @brief: Methods for reliable transfer of data packets
 *
 * @author: Evan Beachly
 * @date: 11/3/2015 12:24 PM
 * 
 */

#include "rdt.hpp"


/*** Protocol Definitions ***/
#define START_BYTE 0xEB
#define HEADER_PLUS_TRAILER_LENGTH 5

#define START_BYTE_INDEX 0
#define DATA_LENGTH_INDEX 1
#define SEQUENCE_NUM_INDEX 2
#define DATA_INDEX 3
#define CHECKSUM_MSB_INDEX -2
#define CHECKSUM_LSB_INDEX -1

/*** GLOBALS ***/
//Buffer to construct the packet in. Must be larger than 256 + HEADER_PLUS_TRAILER_LENGTH
#define BUFFER_SIZE 300
uint8_t rdt_readBytes[BUFFER_SIZE];
uint16_t rdt_readBytesLength = 0;
uint8_t rdt_dataBytes[256];
Packet rdt_receivedPacket = {0, 0, rdt_dataBytes, 0};
Packet rdt_transmittedPacket;
uint8_t rdt_maxAllowedDataLength = 255;

uint16_t rdt_rxSequenceId = 0xFFFF;
uint8_t rdt_txSequenceId = 0;

void setMaxAllowedDataLength(uint8_t maxAllowedDataLength)
{
    rdt_maxAllowedDataLength = maxAllowedDataLength;
}

/*
 * @brief Determines if a packet is an acknowledgement packet or a data packet
 * @param packet the packet to check
 * @return 1 if the packet is an Acknowledgement. 0 if the packet is a data packet
 */
int isAck(const Packet* ackPacket ){
    if ( ackPacket->dataLength == 0 ){
        return 1;
    }
    return 0;
}

/*
 * @brief Determines if an acknowledgement packet is acking a packet you transmitted
 * @param transmittedPacketsCrc16 the 16 bit crc of the packet you transmitted
 * @param ackPacket the acknowledgement packet. Verify this is an ack with isAck
 * @return 1 if this Ack is acknowledging the transmitted packet. 0 if it's acknowledging some other packet.
 */
int isAcking( const Packet* transmittedPacket, const Packet* ackPacket )
{
    if ( ackPacket->crc16 == transmittedPacket->crc16 &&
        ackPacket->sequenceId == transmittedPacket->sequenceId)
    {
        return 1;
    }
    return 0;
}


void transmitAck( uint16_t crc16, uint8_t sequenceId, Serial* serial );
uint16_t calculateCrc( const uint8_t* buffer, int offset, int length);

//Modify this function depending on your hardware
//Transmits a single byte of the serial connection
inline void sendByte(uint8_t byt, Serial* serial)
{
    serial->writeByte(byt);
    return;
}

//Modify this function depending on your hardware
//Non-blocking. Returns 1 if a byte was received, and sets byt to the value. Returns 0 if no byte was received.
inline uint8_t receiveByte(uint8_t* byt, Serial* serial)
{
    return serial->readByte(byt);
}

//Returns NULL if no packet ready. Returns pointer to packet if a packet is ready.
//Acknowledgement packets have 0 data length, and the crc is the crc of the ack'd packet.
const Packet* checkForPacket( Serial* serial )
{
  uint16_t calcCrc16;
  uint16_t readCrc16;
  uint16_t packetLength;
  uint8_t readByte;
  Packet* ret = NULL;

  uint16_t numRemainingBufferedBytes;
  uint16_t i;
  uint8_t isCorrupted = 0;

  //Receive bytes until the buffer is full or the stream is empty
  while (rdt_readBytesLength < BUFFER_SIZE && receiveByte(&readByte, serial))
  {
    //Copy the byte if we've already started receiving of a packet,
    //or if it's a start byte
    if (rdt_readBytesLength || readByte == START_BYTE)
    {
      rdt_readBytes[rdt_readBytesLength] = readByte;
      ++rdt_readBytesLength;
    }
  }

  //Do we have enough bytes for a complete packet?
  if (rdt_readBytesLength >= HEADER_PLUS_TRAILER_LENGTH)
  {
    packetLength = 0;
    //Is this an allowed data length?
    if (rdt_readBytes[DATA_LENGTH_INDEX] > rdt_maxAllowedDataLength)
    {
        //Assume the data was corrupted
        isCorrupted = 1;
    }
    //Do we have enough bytes for this packet's data length?
    else if ( rdt_readBytesLength >= (rdt_readBytes[DATA_LENGTH_INDEX] + HEADER_PLUS_TRAILER_LENGTH) )
    {
        packetLength = rdt_readBytes[DATA_LENGTH_INDEX] + HEADER_PLUS_TRAILER_LENGTH;
        //Read the CRC
        readCrc16 = rdt_readBytes[packetLength + CHECKSUM_MSB_INDEX];
        readCrc16 = readCrc16 << 8;
        readCrc16 = readCrc16 | rdt_readBytes[packetLength + CHECKSUM_LSB_INDEX];

        //Check if this packet is an acknowledgement
        if ( rdt_readBytes[DATA_LENGTH_INDEX] == 0 )
        {
          //This is an acknowledgement. Return the packet without validating the
          //checksum or acking the ack.
          rdt_receivedPacket.dataLength = rdt_readBytes[DATA_LENGTH_INDEX];
          rdt_receivedPacket.sequenceId = rdt_readBytes[SEQUENCE_NUM_INDEX];
          //rdt_receivedPacket.data = &rdt_dataBytes[0];
          rdt_receivedPacket.crc16 = readCrc16;
          ret = &rdt_receivedPacket;
        }
        else
        {
          //Not an acknowledgement.

          //Verify the checksum
          calcCrc16 = calculateCrc(&rdt_readBytes[DATA_INDEX], 0, rdt_readBytes[DATA_LENGTH_INDEX]);
          
          if (readCrc16 == calcCrc16)
          {
            //A complete packet

            //Compare the sequence Id to detect duplicate packets.
            if (rdt_rxSequenceId == rdt_readBytes[SEQUENCE_NUM_INDEX])
            {
              //Duplicate packet. Throw away
            }
            else
            {
              //A valid packet

              //Return the packet structure
              rdt_receivedPacket.dataLength = rdt_readBytes[DATA_LENGTH_INDEX];
              rdt_receivedPacket.sequenceId = rdt_readBytes[SEQUENCE_NUM_INDEX];
              //rdt_receivedPacket.data = &rdt_dataBytes[0];
              rdt_receivedPacket.crc16 = readCrc16;
              //Copy the data
              for (i = 0; i < rdt_receivedPacket.dataLength; ++i)
              {
                rdt_dataBytes[i] = rdt_readBytes[DATA_INDEX + i];
              }

              //Update the received sequence id
              rdt_rxSequenceId = rdt_receivedPacket.sequenceId;
              ret = &rdt_receivedPacket;
            }

            //Acknowledge
            transmitAck(readCrc16, rdt_readBytes[SEQUENCE_NUM_INDEX], serial);
          }
          else
          {
            //Invalid Checksum.
            //Corrupted data
            isCorrupted = 1;
          }
        }
    }

    //If this was not an allowed data length, or the checksum was invalid, the data may have been corrupted
    if (isCorrupted)
    {
        //Look for the next start byte
        for (packetLength = 1; packetLength < rdt_readBytesLength; ++packetLength)
        {
          if (rdt_readBytes[packetLength] == START_BYTE)
          {
            //printf("It happened! Index: %u. Data Length: %u\n", packetLength, rdt_readBytes[DATA_LENGTH_INDEX]);
            break;
          }
        }
    }

    //Shift unprocessed bytes to the front
    numRemainingBufferedBytes = rdt_readBytesLength - packetLength;
    for (rdt_readBytesLength = 0; rdt_readBytesLength < numRemainingBufferedBytes; ++rdt_readBytesLength)
    {
        rdt_readBytes[rdt_readBytesLength] = rdt_readBytes[rdt_readBytesLength + packetLength];
    }
    
  }
  else
  {
    //Incomplete packet. Need more data before we can return it.
  }

  return ret;
}

// Transmits an Acknowledgement packet. This has 0 data length, but the crc is the crc of the acknowledged packet.
void transmitAck( uint16_t crc16, uint8_t sequenceId, Serial* serial  )
{
    //First byte is start Byte
    sendByte(START_BYTE, serial);
    //Second byte is dataLength
    sendByte(0, serial);
    //Third byte is sequenceId
    sendByte(sequenceId, serial);
    //Finally, the crc
    sendByte((uint8_t)(crc16 >> 8), serial);
    sendByte((uint8_t)crc16, serial);
}

//Transmits the previous packet.
//Doesn't increment the sequence id, so multiples will be discarded.
const Packet* retryTransmission(Serial* serial)
{
    if (rdt_transmittedPacket.dataLength == 0)
    {
        return NULL;
    }
    //Transmit it
    //First byte is start Byte
    sendByte(START_BYTE, serial);
    //Second byte is dataLength
    sendByte(rdt_transmittedPacket.dataLength, serial);
    //Third byte is sequenceId
    sendByte(rdt_transmittedPacket.sequenceId, serial);
    //Next, the data
    for (int i = 0; i < rdt_transmittedPacket.dataLength; ++i )
    {
        sendByte(rdt_transmittedPacket.data[i], serial);
    }
    //Finally, the crc
    sendByte((uint8_t)(rdt_transmittedPacket.crc16 >> 8), serial);
    sendByte((uint8_t)rdt_transmittedPacket.crc16, serial);

    return &rdt_transmittedPacket;
}

//Transmits a packet containing data. Returns the crc16 of the data.
//This crc16 can be used to verify acknowledgement of the data.
//Acknowledgement packets have 0 data length, but the crc is the crc of the ack'd packet.
const Packet* transmitPacket( const uint8_t* data, uint8_t dataLength, Serial* serial )
{
    if (data == NULL || dataLength == 0)
    {
        return NULL;
    }
    rdt_transmittedPacket.sequenceId = rdt_txSequenceId;
    //Increment the sequence id
    ++rdt_txSequenceId;

    rdt_transmittedPacket.dataLength = dataLength;
    rdt_transmittedPacket.data = data;
    //Calculate the crc
    rdt_transmittedPacket.crc16 = calculateCrc(data, 0, dataLength);

    return retryTransmission(serial);
}

const Packet* transmitStringPacket( const char* str, Serial* serial )
{
    if (str == NULL)
    {
        return NULL;
    }

    int strLength = strlen(str) + 1;
    if (strLength > 255)
    {
        return NULL;
    }

    return transmitPacket((const uint8_t*) str, strLength, serial);
}

/*****************************************************************/
/*                                                               */
/* CRC LOOKUP TABLE                                              */
/* ================                                              */
/* The following CRC lookup table was generated automagically    */
/* by the Rocksoft^tm Model CRC Algorithm Table Generation       */
/* Program V1.0 using the following model parameters:            */
/*                                                               */
/*    Width   : 2 bytes.                                         */
/*    Poly    : 0x8005                                           */
/*    Reverse : FALSE.                                           */
/*                                                               */
/* For more information on the Rocksoft^tm Model CRC Algorithm,  */
/* see the document titled "A Painless Guide to CRC Error        */
/* Detection Algorithms" by Ross Williams                        */
/* (ross@guest.adelaide.edu.au.). This document is likely to be  */
/* in the FTP archive "ftp.adelaide.edu.au/turtlePub/rocksoft".  */
/*                                                               */
/*****************************************************************/

static const uint16_t crctable[256] =
{
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

/*****************************************************************/
/*                   End of CRC Lookup Table                     */
/*****************************************************************/

/*
 * Calculates the crc 16 over a given portion of a buffer, defined by offset and length
 *
 * Width: 16
 * Polynomial: X^16 + X^15 + X^2 + 1
 * Generator: 0x8005
 * Init: 0xFFFF
 * RefIn: FALSE
 * RefOut: FALSE
 * XorOut: 0x0000
 */
uint16_t calculateCrc( const uint8_t* buffer, int offset, int length){

    int i; // Byte index into buffer
    uint8_t k; // Index into look-up table

    // Initialize the CRC value
    uint16_t CRC_value = 0xFFFF;

    // Loop for each byte in the buffer
    for (i=0; i<length; ++i)
    {
        // Compute index into look-up table for the current byte
        k = ( (CRC_value>>8) ^ buffer[offset + i] ) & 0xFF;
        // Update the running CRC_value for the current byte
        CRC_value = (CRC_value << 8) ^ crctable[k];
    }
    return CRC_value;
}