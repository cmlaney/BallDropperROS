/*
 * @brief: Methods for reliably transmitting data packets over serial.
 * Uses a 16 bit CRC to guarantee that any packets received are valid.
 * Corrupted data is thrown away.
 *
 * @author: Evan Beachly
 * @date: 9/3/2016 2:52 PM
 * 
 */

#ifndef RDT_HPP
#define RDT_HPP

#include "stdint.h"
#include "Serial.hpp"

//A datastructure for a packet.
typedef struct packet_
{
    uint8_t dataLength;
    uint8_t* data;
    uint16_t crc16;
} Packet;

/*
 * @brief Non-blocking function to receive data packets over serial.
 * May return Acknowledgement packets, which have 0 data length, and the crc is the crc of the acknowledged packet.
 * Automatically sends Acknowledgement packets in response to non-ack packets.
 * 
 * @return The next packet that has been received, or NULL if no packet ready.
 */
Packet* checkForPacket(Serial* serial);

/*
 * @brief Non-blocking function to receive data packets over serial.
 * May return Acknowledgement packets, which have 0 data length, and the crc is the crc of the acknowledged packet.
 * Does not send acknowledgement packets
 * 
 * @return The next packet that has been received, or NULL if no packet ready.
 */
//Packet* checkForPacketDontAck(Serial* serial);

/*
 * @brief Transmits a data packet over the serial connection
 * @param data a pointer to a byte buffer containing the data to be transmitted
 * @param dataLength the length of the data to transmit
 * @return the 16 bit crc of the data, to be used when checking if the packet was acknowledged.
 */
uint16_t transmitPacket( const uint8_t* data, uint8_t dataLength, Serial* serial );

/*
 * @brief Transmits a packet containing a string over the serial connection
 * @param str A nul-terminated string to transmit.
 * @return the 16 bit crc of the data, to be used when checking if the packet was acknowledged.
 */
uint16_t transmitStringPacket( const char* str, Serial* serial );

/*
 * @brief Packs a uint16_t into a byte buffer in network order.
 * @param data the buffer to write into
 * @param offset offset into the buffer to write
 * @param val the value to write
 */
inline void writeUint16(uint8_t* data, int offset, uint16_t val)
{
    data[offset] = 0xff & (val >> 8);
    data[offset + 1] = 0xff & val;
}

/*
 * @brief Packs a uint32_t into a byte buffer in network order.
 * @param data the buffer to write into
 * @param offset offset into the buffer to write
 * @param val the value to write
 */
inline void writeUint32(uint8_t* data, int offset, uint32_t val)
{
    data[offset] = 0xff & (val >> 24);
    data[offset + 1] = 0xff & (val >> 16);
    data[offset + 2] = 0xff & (val >> 8);
    data[offset + 3] = 0xff & val;
}

/*
 * @brief Reads a network-order uint16_t from a byte buffer
 * @param data the buffer to read from
 * @param offset offset into the buffer to read
 * @returns the read value
 */
inline uint16_t readUint16(uint8_t* data, int offset){
    uint16_t ret;
    ret = data[offset];
    ret = (ret << 8) + data[offset + 1];
    return ret;
}

/*
 * @brief Reads a network-order uint32_t from a byte buffer
 * @param data the buffer to read from
 * @param offset offset into the buffer to read
 * @returns the read value
 */
inline uint32_t readUint32(uint8_t* data, int offset)
{
    uint32_t ret;
    ret = data[offset];
    ret = (ret << 8) + data[offset + 1];
    ret = (ret << 8) + data[offset + 2];
    ret = (ret << 8) + data[offset + 3];
    return ret;
}

/*
 * @brief Determines if a packet is an acknowledgement packet or a data packet
 * @param packet the packet to check
 * @return 1 if the packet is an Acknowledgement. 0 if the packet is a data packet
 */
int isAck(const Packet* ackPacket );

/*
 * @brief Determines if an acknowledgement packet is acking a packet you transmitted
 * @param transmittedPacketsCrc16 the 16 bit crc of the packet you transmitted
 * @param ackPacket the acknowledgement packet. Verify this is an ack with isAck
 * @return 1 if this Ack is acknowledging the transmitted packet. 0 if it's acknowledging some other packet.
 */
int isAcking( uint16_t transmittedPacketsCrc16, const Packet* ackPacket );

#endif