/*************************************************************************
 * 
 *    Code copied from user bitfixer from the romulator project, 
 *     - https://github.com/bitfixer/bf-romulator
 * 
 *************************************************************************
#include "xmodem.h"
#include <Arduino.h>
#include <LittleFS.h>
#include "wtmSerialx.h"

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define ETB 0x17
#define CAN 0x18


extern wtmSerialX* xmodemUart;

#ifdef XMDEBUG
extern wtmSerialX* logger;
#endif

uint8_t calc_checksum(char* ptr, int count)
{
    uint8_t checksum = 0;
    for (int i = 0; i < count; i++)
    {
        checksum += ptr[i];
    }
    return checksum;
}

#ifdef XMDEBUG
void dumpPacket(uint8_t packet[], unsigned int pktLen)
{
    for(unsigned int byteNum = 0; byteNum < pktLen; byteNum++)
    {
        if (byteNum%16 == 0)
        {
            logger->printf("%s%04x:", byteNum==0?"":"\r\n", byteNum);
        }
        logger->printf(" %02x", packet[byteNum]);
    }
    logger->printf("\r\n");
}
#endif

void xmodemRecvFile(const char* fname)
{
    File fp = LittleFS.open(fname, "w");
    uint8_t packet[XM_PKT_SIZE];
    memset(packet, 0, XM_PKT_SIZE);

    while (packet[0] != SOH)
    {
#ifdef XMDEBUG
        logger->printf("1: Write NAK and then receive 132 bytes\r\n");
#endif
        packet[0] = NAK;
        xmodemUart->write(packet, 1);
#ifdef XMDEBUG
        uint8_t bytesRcvd = xmodemUart->readBytes(packet, XM_PKT_SIZE);
        logger->printf("bytesRcvd = %d\r\n", bytesRcvd);
        dumpPacket(packet, bytesRcvd);
#else
        xmodemUart->readBytes(packet, XM_PKT_SIZE);
#endif
    }

    bool done = false;
    int bytesRead = 0;
    int repeatedPackets = 0;
    uint8_t prevPacketNumber = 0;
    uint8_t expectedPacketNumber = 1;
    while (!done && (repeatedPackets < 100))
    {
        if (packet[0] == EOT)
        {
#ifdef XMDEBUG
            logger->printf("2: EOT received. Send ACK\r\n");
#endif
            packet[0] = ACK;
            xmodemUart->write(packet, 1);
            done = true;
            continue;
        }

        // first check if expected packet number matches
        uint8_t packetNum = packet[1];
        uint8_t invPacketNum = packet[2];
        uint8_t checksum = calc_checksum((char*)&packet[3], 128);
        // check checksum value
#ifdef XMDEBUG
        logger->printf("3: packetNum = packet[1] = 0x%02x, invPacketNum = packet[2] = 0x%02x\r\n", packetNum, invPacketNum);
        logger->printf("3: packet[131] = 0x%02x, packetNum + invPacketNum = 0x%02x, checksum = 0x%02x\r\r\n", packet[131], packetNum + invPacketNum, checksum);
        logger->printf("3: packetNum = 0x%02x, expectedPacketNumber = 0x%02x, prevPacketNumber = 0x%02x\r\n", packetNum, expectedPacketNumber, prevPacketNumber);
        dumpPacket(packet, 132);
#endif
        if (checksum == packet[131] && packetNum + invPacketNum == 0xFF)
        {
            if (packetNum == expectedPacketNumber)
            {
#ifdef XMDEBUG
                logger->printf("3: packetNum == expectedPacketNumber, send ACK, save segment to file\r\n");
#endif
                // write to file
                bytesRead += 128;
                fp.write(&packet[3], 128);
                packet[0] = ACK;
                xmodemUart->write(packet, 1);
                prevPacketNumber = packetNum;
                expectedPacketNumber = prevPacketNumber + 1;
            }
            else if (packetNum == prevPacketNumber)
            {
#ifdef XMDEBUG
                logger->printf("3: packetNum == prevPacketNumber, send ACK\r\n");
#endif
                // last packet repeated, ack and continue
                repeatedPackets++;
                packet[0] = ACK;
                xmodemUart->write(packet, 1);
            }
        }
        else
        {
#ifdef XMDEBUG
            logger->printf("3: packetNum == something else, send NACK\r\n");
#endif
            repeatedPackets++;
            packet[0] = NAK;
            xmodemUart->write(packet, 1);
        }
#ifdef XMDEBUG
        logger->printf("4: Read another 132 bytes\r\n");
        uint8_t bytesRcvd = xmodemUart->readBytes(packet, XM_PKT_SIZE);
        logger->printf("bytesRcvd = %d\r\n", bytesRcvd);
#else
        xmodemUart->readBytes(packet, XM_PKT_SIZE);
#endif
    }
    fp.close();
    delay(500);
    xmodemUart->printf("read %d bytes, repeated packets %d\r\n", bytesRead, repeatedPackets);
    xmodemUart->fileTransferMode = false;

}