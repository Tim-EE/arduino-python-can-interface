// this sketch turn an Arduino and a MCP2515 CAN module into a Python-CAN compatible interface
// see: https://python-can.readthedocs.io/en/master/interfaces/serial.html for serial interface

#include <CAN.h>
#include <cppQueue.h>

#define USE_SOFT_SERIAL
//#define DEBUG_FSM
#ifdef DEBUG_FSM
#ifndef USE_SOFT_SERIAL
#define USE_SOFT_SERIAL
#endif
#endif

#ifdef USE_SOFT_SERIAL
#include <SoftwareSerial.h>
#endif

#define FRAME_START 0xAA
#define FRAME_END   0xBB
#define EXTENDED_ID_FLAG 0x80000000
#define PAYLOAD_OFFSET 6

/** Serial state machine states */
typedef enum {START_STATE,TIMESTAMP_STATE,DLC_STATE,ARBITRATION_ID_STATE,PAYLOAD_STATE,END_STATE} SerialRxFSM;

typedef struct
{
  uint8_t timestamp[4];
  uint8_t dlc;
  bool is_extended_id = false;
  uint8_t arbitration_id[4];
  uint8_t payload[8];
} SerialCanPacket;

SerialCanPacket txMessage; // message to transmit over CAN bus
SerialCanPacket rxMessage; // message received over CAN bus

cppQueue  toCanQueue(sizeof(SerialCanPacket), 32, LIFO); // queue to store serial data going to CAN bus
#ifdef USE_SOFT_SERIAL
SoftwareSerial softSerial(-1, 7); // RX, TX
void printSerialCanPacket(SerialCanPacket* frame);
#endif

void sendPyCanFrameToSerial(SerialCanPacket* frame)
{
  *(uint32_t*)frame->timestamp = millis();
  Serial.write(FRAME_START);
  Serial.write(frame->timestamp,4);
  Serial.write(frame->dlc);
  Serial.write(frame->arbitration_id,3);
  if (frame->is_extended_id)
    Serial.write(frame->arbitration_id[3] | 0x80);
  else
    Serial.write(frame->arbitration_id[3]);
  Serial.write(frame->payload,frame->dlc);
  Serial.write(FRAME_END);
}

void sendPyCanFrameToCanBus(SerialCanPacket* frame)
{
    if (frame->is_extended_id)
      CAN.beginExtendedPacket(*((uint32_t*)frame->arbitration_id));
    else
      CAN.beginPacket(*((uint32_t*)frame->arbitration_id));
    CAN.write(frame->payload,frame->dlc);
    CAN.endPacket();
    sendPyCanFrameToSerial(frame);
}



bool getMessageFromSerial(SerialCanPacket* frame)
{
  static uint8_t byteCount = 0;
  static bool serialTransferInProgress = false;
  static SerialRxFSM fsmState = START_STATE;
  while (Serial.available() > 0)
  {
    uint8_t serialByte = Serial.read();
    switch(fsmState)
    {
      case START_STATE:
        if (serialByte == FRAME_START)
        {
          fsmState = TIMESTAMP_STATE;
          byteCount = 0;
          serialTransferInProgress = true;
#ifdef DEBUG_FSM
          softSerial.println("    -SOF found!");
#endif
        }
        break;
        
      case TIMESTAMP_STATE:
        frame->timestamp[byteCount] = serialByte;
        
#ifdef DEBUG_FSM
        softSerial.print("    -TIMESTAMP[");
        softSerial.print(byteCount);
        softSerial.print("]: ");
        softSerial.println(frame->timestamp[byteCount]);
#endif        
        
        byteCount++;

        if (byteCount == 4)
        {
          fsmState = DLC_STATE;
          byteCount = 0;
#ifdef DEBUG_FSM
          softSerial.print("    -TIMESTAMP:");
          softSerial.println(*((uint32_t*)&frame->timestamp));
#endif          
        }
        break;
        
      case DLC_STATE:
        frame->dlc = serialByte;

#ifdef DEBUG_FSM
        softSerial.print("    -DLC:");
        softSerial.println(frame->dlc);
#endif
        
        byteCount = 0;
        fsmState = ARBITRATION_ID_STATE;
        if (frame->dlc > 8)
        {
          byteCount = 0;
          fsmState = START_STATE;

#ifdef DEBUG_FSM
          softSerial.print("    -ERROR: moving to START_STATE");
#endif
          serialTransferInProgress = false;
          return false;
        }
        
        break;

      case ARBITRATION_ID_STATE:
        frame->arbitration_id[byteCount] = serialByte;
#ifdef DEBUG_FSM
        softSerial.print("    -arbitration_id[");
        softSerial.print(byteCount);
        softSerial.print("]: ");
        softSerial.println(frame->arbitration_id[byteCount]);
#endif
        
        byteCount++;
        if (byteCount == 4)
        {
          if (serialByte & 0x80)
          {
            frame->is_extended_id = true;
#ifdef DEBUG_FSM
            softSerial.print("    -is extended ID ");
            softSerial.println(serialByte);
#endif
          }
          else
          {
            frame->is_extended_id = false;
          }

          // mask out >29-bits in the arbitration ID
          frame->arbitration_id[3] = serialByte & 0x01;
          
          if (frame->dlc == 0)
            fsmState = END_STATE;
          else
            fsmState = PAYLOAD_STATE;
          byteCount = 0;
#ifdef DEBUG_FSM
          softSerial.println("    -moving to PAYLOAD_STATE");
#endif
        }
        break;
      case PAYLOAD_STATE:
        frame->payload[byteCount] = serialByte;

#ifdef DEBUG_FSM
        softSerial.print("    -payload[");
        softSerial.print(byteCount);
        softSerial.print("]: ");
        softSerial.println(frame->payload[byteCount]);
#endif

        byteCount++;
        
        if (byteCount >= frame->dlc)
        {
          fsmState = END_STATE;
#ifdef DEBUG_FSM
          softSerial.println("    -moving to END_STATE");
#endif
        }
        break;
        
      case END_STATE:
        if (serialByte == FRAME_END)
        {
          serialTransferInProgress= false;
          byteCount = 0;
          fsmState = START_STATE;
#ifdef DEBUG_FSM
          softSerial.println("    -message complete");
          softSerial.println("    -moving to START_STATE");
#endif
          return true;
        }
        break;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
#ifdef USE_SOFT_SERIAL
  softSerial.begin(115200);
  while (!softSerial);
  softSerial.println("-------------------------");
  softSerial.println("ready.");
#endif
  
  CAN.setPins(10);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3))
  {
    
#ifdef USE_SOFT_SERIAL
    softSerial.println("error.");
#endif
    pinMode(LED_BUILTIN, OUTPUT);
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }
}

void loop()
{
  if (!toCanQueue.isEmpty())
  {
    SerialCanPacket tmpMsg;
    toCanQueue.pop(&tmpMsg);
    sendPyCanFrameToCanBus(&tmpMsg);
  }

  // try to parse packet
  int packetSize = CAN.parsePacket();
  if (packetSize)
  {
    rxMessage.is_extended_id=CAN.packetExtended();
    if (CAN.packetExtended())
      *((uint32_t*)rxMessage.arbitration_id) = CAN.packetId() + EXTENDED_ID_FLAG;
    else
      *((uint32_t*)rxMessage.arbitration_id) = CAN.packetId();
    
    rxMessage.dlc = CAN.packetDlc();
    for (byte i = 0; i < rxMessage.dlc; i++)
    {
      rxMessage.payload[i] = CAN.read();
    }
    
    sendPyCanFrameToSerial(&rxMessage);
  }

  // process serial data, return true if a full packet has been parsed
  if (getMessageFromSerial(&txMessage))
  {
    // if we get here, txMessage has been populated with data from the Python-CAN serial port
    // push the txMessage data onto the queue
    toCanQueue.push(&txMessage);
#ifdef USE_SOFT_SERIAL
    softSerial.println(*((uint32_t*)&txMessage.arbitration_id));
    //printSerialCanPacket(&txMessage);
#endif
  }
}

#ifdef USE_SOFT_SERIAL
void printSerialCanPacket(SerialCanPacket* frame)
{
  softSerial.print("  timestamp:\t");
  softSerial.println(*((uint32_t*)&txMessage.timestamp));
  softSerial.print("  dlc:\t\t");
  softSerial.println(*((uint8_t*)&txMessage.dlc));
  softSerial.print("  arb_id:\t");
  softSerial.println(*((uint32_t*)&txMessage.arbitration_id));
  softSerial.println("  payload:");
  for (int i = 0; i < 8;i++)
  {
    softSerial.print("    ");
    softSerial.print(i);
    softSerial.print(":\t");
    softSerial.println(txMessage.payload[i]);
  }
}
#endif
