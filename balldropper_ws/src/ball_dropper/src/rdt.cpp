#include "rdt.hpp"


//Buffer to construct the packet in
uint8_t readBytes[262];
int readBytesLength = 0;
Packet ret;

char hexToAscii[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

static const uint8_t startByte = 0xEB;

/*
 * @brief Determines if a packet is an acknowledgement packet or a data packet
 * @param packet the packet to check
 * @return 1 if the packet is an Acknowledgement. 0 if the packet is a data packet
 */
int isAck(const Packet* ackPacket ){
    if ( ackPacket->dataLength == 0 && ackPacket->crc16 != 0xFFFF ){
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
int isAcking( uint16_t transmittedPacketsCrc16, const Packet* ackPacket )
{
    if ( ackPacket->crc16 == transmittedPacketsCrc16 )
    {
        return 1;
    }
    return 0;
}


void transmitAck( uint16_t crc16, Serial* serial );
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
Packet* checkForPacketDontAck( Serial* serial )
{
    uint16_t calcCrc16;
    uint16_t readCrc16;
    uint8_t dataAvailable;
    uint8_t readByte;
    //Check for data available
    dataAvailable = receiveByte(&readByte, serial);
    if (dataAvailable)
    {
        printf("%c%c ", hexToAscii[readByte >> 4], hexToAscii[readByte & 0x0F]);
        fflush(stdout);
        //Check if we aren't in the middle of a packet
        if (readBytesLength == 0 )
        {
            //Check that this is the start of a new packet
            if (readByte == startByte)
            {
                //Copy it into the buffer
                readBytes[readBytesLength] = readByte;
                readBytes[1] = 0;
                ++readBytesLength;
            }
            else
            {
                //Unexpected start byte. Throw it away
                printf("Unexpected Start Byte: %d\n", readByte);
                return NULL;
            }
        }
        else  //We are in the middle of a packet
        {
            //Copy the byte
            readBytes[readBytesLength] = readByte;
            ++readBytesLength;
        }
        //Read until the end of the packet or the end of the buffer
        while ( (readBytesLength < (readBytes[1] + 4)) &&
                ((dataAvailable = receiveByte(&readByte, serial)) != 0)
                )
        {
            //Copy the byte
            readBytes[readBytesLength] = readByte;
            ++readBytesLength;
        }
        //Is this packet complete?
        if (readBytesLength == (readBytes[1] + 4))
        {
            //Read the CRC
            readCrc16 = readBytes[readBytesLength - 2];
            readCrc16 = readCrc16 << 8;
            readCrc16 = readCrc16 | readBytes[readBytesLength -1];

            //Indicate that this packet is complete, so we can get the next one.
            readBytesLength = 0;

            //Check if this packet is an acknowledgement
            if ( readBytes[1] == 0 && readCrc16 != 0xFFFF )
            {
                //This is an acknowledgement. Return the packet without validating the checksum or acking the ack.
                ret.dataLength = readBytes[1];
                ret.data = &readBytes[2];
                ret.crc16 = readCrc16;
                return &ret;
            }
            //Verify the checksum
            calcCrc16 = calculateCrc(&readBytes[2], 0, readBytes[1]);

            if (readCrc16 == calcCrc16)
            {
                //A complete, valid packet. Return the packet structure
                ret.dataLength = readBytes[1];
                ret.data = &readBytes[2];
                ret.crc16 = readCrc16;

                //printf("Data Packet Received: %s\n", ret.data);
                return &ret;
            }
            else
            {
                //Invalid Checksum. Throw away the packet
                if (readBytes[1] > 0)
                {
                    printf("Invalid Checksum over [%s]", &readBytes[2]);
                }
                else
                {
                    printf("Invalid Checksum\n");
                }

                return NULL;
            }
        }
        //Incomplete packet. Need more data before we can return it.
        return NULL;
    }
    else
    {
        //Nothing read, so nothing to do
        return NULL;
    }
}

Packet* checkForPacket( Serial* serial )
{
    Packet* receivedPacket;
    receivedPacket = checkForPacketDontAck(serial);
    if ((receivedPacket != NULL) && !isAck(receivedPacket))
    {
        transmitAck(receivedPacket->crc16, serial);
    }
    return receivedPacket;
}

// Transmits an Acknowledgement packet. This has 0 data length, but the crc is the crc of the acknowledged packet.
void transmitAck( uint16_t crc16, Serial* serial  )
{
    //First byte is start Byte
    sendByte(startByte, serial);
    //Second byte is dataLength
    sendByte(0, serial);
    //Finally, the crc
    sendByte((uint8_t)(crc16 >> 8), serial);
    sendByte((uint8_t)crc16, serial);
}

//Transmits a packet containing data. Returns the crc16 of the data.
//This crc16 can be used to verify acknowledgement of the data.
//Acknowledgement packets have 0 data length, but the crc is the crc of the ack'd packet.
uint16_t transmitPacket( const uint8_t* data, uint8_t dataLength, Serial* serial )
{
    if (data == NULL)
    {
        return 0;
    }

    //Calculate the crc
    uint16_t crc16 = calculateCrc(data, 0, dataLength);

    //Transmit it
    //First byte is start Byte
    sendByte(startByte, serial);
    //Second byte is dataLength
    sendByte(dataLength, serial);
    //Next, the data
    for (int i = 0; i < dataLength; ++i )
    {
        sendByte(data[i], serial);
    }
    //Finally, the crc
    sendByte((uint8_t)(crc16 >> 8), serial);
    sendByte((uint8_t)crc16, serial);
    return crc16;
}

uint16_t transmitStringPacket( const char* str, Serial* serial )
{
    if (str == NULL)
    {
        return 0;
    }

    int strLength = strlen(str) + 1;
    if (strLength > 255)
    {
        return 0;
    }

    //Calculate the crc
    uint16_t crc16 = calculateCrc((uint8_t*)str, 0, strLength);
    //Transmit it
    //First byte is start Byte
    sendByte(startByte, serial);
    //Second byte is dataLength
    sendByte(strLength, serial);
    //Next, the data
    for (int i = 0; i < strLength; ++i )
    {
        sendByte(str[i], serial);
    }
    //Finally, the crc
    sendByte((uint8_t)(crc16 >> 8), serial);
    sendByte((uint8_t)crc16, serial);
    return crc16;
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
/* in the FTP archive "ftp.adelaide.edu.au/turtlePub/rocksoft".        */
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