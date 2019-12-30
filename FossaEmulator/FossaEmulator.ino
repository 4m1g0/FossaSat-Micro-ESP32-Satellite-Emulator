#include <Arduino.h>
#include <RadioLib.h>
#include "src/Comms/Comms.h"

// pin definition
#define LORA_NSS 18
#define LORA_DI00 26
#define LORA_DI01 12
// modem configuration
#define LORA_CARRIER_FREQUENCY                          436.7f  // MHz
#define LORA_BANDWIDTH                                  125.0f  // kHz dual sideband
#define LORA_SPREADING_FACTOR                           11
#define LORA_SPREADING_FACTOR_ALT                       10
#define LORA_CODING_RATE                                8       // 4/8, Extended Hamming
#define LORA_OUTPUT_POWER                               21      // dBm
#define LORA_CURRENT_LIMIT                              120     // mA
#define SYNC_WORD_7X                                    0xFF    // sync word when using SX127x
#define SYNC_WORD_6X                                    0x0F0F  //                      SX126x

// satellite callsign
char callsign[] = "FOSSASAT-1";

SX1276 lora = new Module(LORA_NSS, LORA_DI00, LORA_DI01);

bool receivedFlag = false;
bool enableInterrupt = true;

void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void handleReceivedPacket();
void sendPong();

void setup() {
  Serial.begin(115200);
  Serial.print(F("[SX12xx] Initializing ... "));
  int state = lora.begin(LORA_CARRIER_FREQUENCY,
                          LORA_BANDWIDTH,
                          LORA_SPREADING_FACTOR,
                          LORA_CODING_RATE,
                          SYNC_WORD_7X,
                          17,
                          (uint8_t)LORA_CURRENT_LIMIT);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  lora.setDio1Action(setFlag);
  
  // start listening for LoRa packets
  Serial.print(F("[SX12x8] Starting to listen ... "));
  state = lora.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  Serial.println("*****************************************");
  Serial.println("********* List of commands **************");
  Serial.println("'p' - Pong");
  //Serial.println("'s' - Sys Info");
  Serial.println("*****************************************");
}

void loop() {
  // check if the flag is set (received interruption)
  if(receivedFlag) {
    enableInterrupt = false;
    receivedFlag = false;
    handleReceivedPacket();
    enableInterrupt = true;
    lora.startReceive();
  }

  if (Serial.available()) {
    enableInterrupt = false;
    char c = Serial.read();
    Serial.print(c);

    switch (c) {
      case 'p':
        sendPong();
    }

    // dump the serial buffer
    while(Serial.available()) {
      Serial.read();
    }
    enableInterrupt = true;
    lora.startReceive();
  }

}

void handleReceivedPacket() {
  // read received data
  size_t respLen = lora.getPacketLength();
  uint8_t* respFrame = new uint8_t[respLen];
  int state = lora.readData(respFrame, respLen);

  if (state == ERR_NONE) {
    PRINT_BUFF(respFrame, respLen);
  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("[SX12x8] CRC error!"));
  } else {
    // some other error occurred
    Serial.print(F("[SX12x8] Failed, code "));
    Serial.println(state);
  }
}

void sendPong() {
  Serial.print(F("Sending pong frame ... "));
  uint8_t functionId = RESP_PONG;
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);
  int state = lora.transmit(frame, len);
  delete[] frame;

  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}