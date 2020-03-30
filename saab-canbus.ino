/**
 * saab-canbus
 * Reads from and writes to the Instrumentation Bus (I-Bus) on a Saab 9-3 (tested on 2001)
 */

#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN);

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];

// button flags
unsigned char flag_wheelVolDown = 0;
unsigned char flag_wheelVolUp = 0;
unsigned char flag_wheelSrc = 0;
unsigned char flag_wheelSeekRight = 0;
unsigned char flag_wheelSeekLeft = 0;
unsigned char flag_wheelNxt = 0;
unsigned char flag_sidClr = 0;
unsigned char flag_sidSet = 0;
unsigned char flag_sidDown = 0;
unsigned char flag_sidUp = 0;
unsigned char flag_sidNpanel = 0;
unsigned char flag_sidClockPlus = 0;
unsigned char flag_sidClockMin = 0;

// car states
unsigned char state_locked = 0;
unsigned char state_frontLDoor = 0;
unsigned char state_frontRDoor = 0;
unsigned char state_rearLDoor = 0;
unsigned char state_rearRDoor = 0;
unsigned char state_trunk = 0;

void setup()
{
    Serial.begin(115200);
    while (CAN_OK != CAN.begin(CAN_47KBPS))
    { // init can bus : baudrate = 47k
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

    // set the interrupt flag if a message is detected on the bus
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING);
}

/**
 * ISR for setting recv flag
 */
void MCP2515_ISR()
{
    flagRecv = 1;
}

/**
 * main loop
 */
void loop()
{
    if (flagRecv)
    {
        flagRecv = 0; // clear interrupt flag
        handleRecv();
    }
    if (flag_wheelVolDown)
    {
        flag_wheelVolDown = 0; // clear flag
    }
    if (flag_wheelVolUp)
    {
        flag_wheelVolUp = 0; // clear flag
    }
    if (flag_wheelSrc)
    {
        flag_wheelSrc = 0; // clear flag
    }
    if (flag_wheelSeekRight)
    {
        flag_wheelSeekRight = 0; // clear flag
    }
    if (flag_wheelSeekLeft)
    {
        flag_wheelSeekLeft = 0; // clear flag
    }
    if (flag_wheelNxt)
    {
        flag_wheelNxt = 0; // clear flag
    }
    if (flag_sidClr)
    {
        flag_sidClr = 0; // clear flag
    }
    if (flag_sidSet) {
        flag_sidSet = 0; // clear flag
    }
    if (flag_sidDown) {
        flag_sidDown = 0; // clear flag
    }
    if (flag_sidUp) {
        flag_sidUp = 0; // clear flag
    }
    if (flag_sidNpanel) {
        flag_sidNpanel = 0; // clear flag
    }
    if (flag_sidClockPlus) {
        flag_sidClockPlus = 0; // clear flag
    }
    if (flag_sidClockMin) {
        flag_sidClockMin = 0; // clear flag
    }
}

/**
 * Handle an incomming canbus message
 */
void handleRecv()
{
    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();

        // call the appropriate message handler
        switch (canId)
        {
        case 0x290:
            parse290(buf);
            break;
        case 0x320:
            parse320(buf);
            break;
        }
    }
}

/**
 * Parses 0x290 messages (Steering wheel and SID buttons) and sets button flags
 */
void parse290(unsigned char *msgBuf)
{
    char state = msgBuf[0]; // 1 if state has changed, 0 otherwise
    char audio = msgBuf[2]; // audio controls state (duplicated on byte 4)
    char sid = msgBuf[3];   // SID controls state (duplicated on byte 5)

    if (state)
    // only need to update flags if state has changed
    {
        // byte2 is steering wheel
        flag_wheelVolDown |= (audio >> 7) & 1;
        flag_wheelVolUp |= (audio >> 6) & 1;
        flag_wheelSrc |= (audio >> 5) & 1;
        flag_wheelSeekRight |= (audio >> 4) & 1;
        flag_wheelSeekLeft |= (audio >> 3) & 1;
        flag_wheelNxt |= (audio >> 2) & 1;

        // byte3 is sid
        flag_sidClr |= (sid >> 7) & 1;
        flag_sidSet |= (sid >> 6) & 1;
        flag_sidDown |= (sid >> 5) & 1;
        flag_sidUp |= (sid >> 4) & 1;
        flag_sidNpanel |= (sid >> 3) & 1;
        flag_sidClockPlus |= (sid >> 2) & 1;
        flag_sidClockMin |= (sid >> 1) & 1;
    }
}

/**
 * Parses 0x320 messages (doors, central locking, and seat belts?) and updates state variables
 */
void parse320(unsigned char* msgBuf)
{
    char state = msgBuf[0]; // 1 if state has changed, 0 otherwise
    char door = msgBuf[1]; // locked state and door status
    char belt = msgBuf[2]; // related to seat belts, unknown meaning
    char bulbs = msgBuf[4]; // related to broken light bulbs

    state_locked = !(door >> 7) & 1; // 1 is locked, 0 is unlocked (read bit is inverted)
    state_frontLDoor = (door >> 6) & 1; // 1 i opened , 0 is closed
    state_frontRDoor = (door >> 5) & 1; // 1 is opened, 0 is closed
    state_rearLDoor = (door >> 4) & 1; // 1 is opened, 0 is closed
    state_rearRDoor = (door >> 3) & 1; // 1 is opened, 0 is closed
    state_trunk = (door >> 2) & 1; // 1 is opened, 0 is closed
}
