#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_47KBPS))
    {
        Serial.println("CAN BUS Shield initialization failed");
        Serial.println(" Trying again...");
        delay(100);
    }
    Serial.println("CAN BUS Shield initialized!");

    setup_interrupts();
}

void setup_interrupts()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 15624;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  //Serial.println("EMIT CD CHANGER");
  emitCDChangerMessage();
}


void loop()
{
    unsigned char len = 0;
    unsigned char buf[8];

    if(CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, buf);

        unsigned long devId = CAN.getCanId();

        handleBusMessage(devId, len, buf);
        
//        Serial.println("-----------------------------");
//        Serial.print("Get data from ID: 0x");
//        Serial.println(devId, HEX);
//
//        for(int i = 0; i<len; i++)    // print the data
//        {
//            Serial.print(buf[i], HEX);
//            Serial.print("\t");
//        }
//        Serial.println();
    }
}

void emitCDChangerMessage() {
  unsigned char cdChangerMessage[8];

  cdChangerMessage[0] = 0x80; // CHANGED byte
  cdChangerMessage[2] = 0x63; // MAGAZINE byte
  cdChangerMessage[3] = 0x41; // DISC byte
  cdChangerMessage[4] = 0x1;  // TRACK byte
  cdChangerMessage[5] = 0x6;  // MIN byte
  cdChangerMessage[6] = 0x1;  // SEC byte
  cdChangerMessage[7] = 0xD0; // SECURITY byte

  CAN.sendMsgBuf(0x3C8, 0, 8, cdChangerMessage);
}



void handleBusMessage(unsigned long devId, unsigned char len, unsigned char buf[]) {
  if (devId == 0x368) {
    Serial.println(getDeviceNameFromID(devId));
  } else if (devId == 0x290) {
    Serial.println(getDeviceNameFromID(devId));
  } else if (devId == 0x3C0) {
    Serial.println(getDeviceNameFromID(devId));
  }
}

String getDeviceNameFromID(unsigned long devId) {
  switch(devId) {
    case 0x368:
      return "SID text priority";
    case 0x290:
      return "Steering wheel and SID buttons";
    case 0x3C0:
      return "CD Changer Control";
    default:
      return String(devId);
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
