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
unsigned long lastUpdateTime = millis();

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
unsigned int state_dimmer = 0;
unsigned int state_lightSensor = 0;
unsigned char state_npanel = 0;

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

    if (millis() - lastUpdateTime >= 1000) {
        // a second has passed, send interval messages
        lastUpdateTime = millis();
        sendIntervalMsgs();
    }
}

/**
 * Any messages that must be send on a 1 second interval are sent here
 */
void sendIntervalMsgs()
{
    syncCdChanger();
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
        case 0x410:
            parse410(buf);
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

    if (state)
    // only need to update state if state has changed
    {
        state_locked = !(door >> 7) & 1; // 1 is locked, 0 is unlocked (read bit is inverted)
        state_frontLDoor = (door >> 6) & 1; // 1 i opened , 0 is closed
        state_frontRDoor = (door >> 5) & 1; // 1 is opened, 0 is closed
        state_rearLDoor = (door >> 4) & 1; // 1 is opened, 0 is closed
        state_rearRDoor = (door >> 3) & 1; // 1 is opened, 0 is closed
        state_trunk = (door >> 2) & 1; // 1 is opened, 0 is closed
    }
}

/**
 * Parses 0x410 messages (light dimmer and light sensor) and updates state variables
 */
void parse410(unsigned char* msgBuf)
{
    char state = msgBuf[0];
    char dim1 = msgBuf[1];
    char dim0 = msgBuf[2];
    char light1 = msgBuf[3];
    char light0 = msgBuf[4];
    char npanel = msgBuf[5];

    if (state)
    // only need to update state if state has changed
    {
        state_dimmer = (dim1 << 4) || dim0;
        state_lightSensor = (light1 << 4) || light0;
        state_npanel = npanel;
    }
}


/**
 * Send a message to the SID that there is a cd changer present. Stock setup has this message
 * sending every second
 */
void syncCdChanger(void)
{
    unsigned char syncMsg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    syncMsg[0] = 0x80; // set changed bit to 1
    syncMsg[2] = 0x3F; // set magazine byte to all discs occupied
    syncMsg[3] = 0x41; // cd changer is active and playing disc 1
    syncMsg[4] = 0x69; // current track number
    syncMsg[5] = 0x04; // set current minute
    syncMsg[6] = 0x20; // set current second
    syncMsg[7] = 0xD0; // security byte, 0xD0 means everything is GOOD

}
