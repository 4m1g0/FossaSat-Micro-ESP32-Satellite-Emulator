#include <Arduino.h>
#include <RadioLib.h>
#include "src/Comms/Comms.h"

//#define SX126X // uncomment this line when Using SX126X
// pin definition
#define LORA_NSS 18
#define LORA_DI00 26
#define LORA_DI01 12
#define LORA_RST 14
#define LORA_BUSSY 32
// modem configuration
#define LORA_CARRIER_FREQUENCY                          436.7f  // MHz
#define LORA_BANDWIDTH                                  125.0f  // kHz dual sideband
#define LORA_SPREADING_FACTOR                           11
#define LORA_SPREADING_FACTOR_ALT                       10
#define LORA_CODING_RATE                                8       // 4/8, Extended Hamming
#define LORA_OUTPUT_POWER                               17      // dBm
#define LORA_CURRENT_LIMIT                              120     // mA
#define SYNC_WORD                                       0x12    // sync word
#define LORA_PREAMBLE_LENGTH                            8U
#define LORA_TCXO_VOLTAGE                               1.6f

#ifdef SX126X
SX1268 lora = new Module(LORA_NSS, LORA_DI01, LORA_RST, LORA_BUSSY);
#else
SX1276 lora = new Module(LORA_NSS, LORA_DI00, LORA_RST, LORA_DI01);
#endif


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
void sendSysInfo(bool malformed = false);

void setup() {
  Serial.begin(115200);
  Serial.print(F("[SX12xx] Initializing ... "));
#ifdef SX126X
  int state = lora.begin(LORA_CARRIER_FREQUENCY,
                          LORA_BANDWIDTH,
                          LORA_SPREADING_FACTOR,
                          LORA_CODING_RATE,
                          SYNC_WORD,
                          LORA_OUTPUT_POWER,
                          LORA_CURRENT_LIMIT,
                          LORA_PREAMBLE_LENGTH,
                          LORA_TCXO_VOLTAGE);
#else
  int state = lora.begin(LORA_CARRIER_FREQUENCY,
                          LORA_BANDWIDTH,
                          LORA_SPREADING_FACTOR,
                          LORA_CODING_RATE,
                          SYNC_WORD,
                          LORA_OUTPUT_POWER,
                          (uint8_t)LORA_CURRENT_LIMIT,
                          LORA_PREAMBLE_LENGTH);
#endif     

  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  #ifdef SX126X
  lora.setDio1Action(setFlag);
  #else
  lora.setDio0Action(setFlag);
  #endif
  
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
  Serial.println("'s' - Sys Info");
  Serial.println("'m' - malformed packet");
  Serial.println("*****************************************");
}

char callsign[] = "FOSSASIM-2";

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
        break;
      case 's':
        sendSysInfo();
        break;
      case 'm':
        sendSysInfo(true);
        break;
      case 'i':
        sendPacketInfo();
        break;
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
    Serial.println();
  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("[SX12x8] CRC error!"));
  } else {
    // some other error occurred
    Serial.print(F("[SX12x8] Failed, code "));
    Serial.println(state);
  }
}

// string length limit
#define MAX_STRING_LENGTH                               32

#define MAX_IMAGE_PACKET_LENGTH                         128

// optional data length limit
#define MAX_OPT_DATA_LENGTH                             128

// radio buffer length limit
#define MAX_RADIO_BUFFER_LENGTH                         (MAX_STRING_LENGTH + 2 + MAX_OPT_DATA_LENGTH)

int16_t Communication_Send_Response(uint8_t respId, uint8_t* optData = nullptr, size_t optDataLen = 0, bool overrideModem = false) {
  // build response frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t frame[MAX_RADIO_BUFFER_LENGTH];
  FCP_Encode(frame, callsign, respId, optDataLen, optData);

  // send response
  return (lora.transmit(frame, len));
}

void sendPong() {
  Serial.print(F("Sending pong frame ... "));
  int state = Communication_Send_Response(RESP_PONG);
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  Serial.println();
}

void retransmit(uint8_t* optData, size_t optDataLen){
  Serial.print(F("Sending Retransmitted packet frame ... "));
  int state = Communication_Send_Response(RESP_REPEATED_MESSAGE, optData, optDataLen);
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  Serial.println();
}

void sendPacketInfo(){
  Serial.print(F("Sending packet info frame ... "));

  static const uint8_t respOptDataLen = 2*sizeof(uint8_t) + 4*sizeof(uint16_t);
  uint8_t respOptData[respOptDataLen];
  uint8_t* respOptDataPtr = respOptData;

  // SNR
  int8_t snr = random(0, 20);
  memcpy(respOptDataPtr, &snr, sizeof(snr));
  respOptDataPtr += sizeof(snr);

  // RSSI
  uint8_t rssi = random(10, 50);
  memcpy(respOptDataPtr, &rssi, sizeof(rssi));
  respOptDataPtr += sizeof(rssi);

  uint16_t loraValid = random(1, 200);
  memcpy(respOptDataPtr, &loraValid, sizeof(loraValid));
  respOptDataPtr += sizeof(loraValid);

  uint16_t loraInvalid = random(1, 200);
  memcpy(respOptDataPtr, &loraInvalid, sizeof(loraInvalid));
  respOptDataPtr += sizeof(loraInvalid);

  uint16_t fskValid = random(1, 200);
  memcpy(respOptDataPtr, &fskValid, sizeof(fskValid));
  respOptDataPtr += sizeof(fskValid);

  uint16_t fskInvalid = random(1, 200);
  memcpy(respOptDataPtr, &fskInvalid, sizeof(fskInvalid));
  respOptDataPtr += sizeof(fskInvalid);

  int state = Communication_Send_Response(RESP_PACKET_INFO, respOptDataPtr, respOptDataLen);
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  Serial.println();
}


void sendSysInfo(bool malformed) {
    // build response frame
  static const uint8_t optDataLen = 7*sizeof(uint8_t) + 3*sizeof(int16_t) + sizeof(uint16_t) + sizeof(uint32_t);
  uint8_t optData[optDataLen];
  uint8_t* optDataPtr = optData;


  uint8_t mpptOutputVoltage = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &mpptOutputVoltage, sizeof(mpptOutputVoltage));
  optDataPtr += sizeof(mpptOutputVoltage);
  Serial.print("mpptOutputVoltage: "); Serial.println(mpptOutputVoltage);

  int16_t mpptOutputCurrent = ((float)random(-50000, 50000) / 1000000.0) * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  memcpy(optDataPtr, &mpptOutputCurrent, sizeof(mpptOutputCurrent));
  optDataPtr += sizeof(mpptOutputCurrent);
  Serial.print("mpptOutputCurrent: "); Serial.println(mpptOutputCurrent);

  uint32_t onboardTime = 1581619348;
  memcpy(optDataPtr, &onboardTime, sizeof(onboardTime));
  optDataPtr += sizeof(onboardTime);
  Serial.print("onboardTime: "); Serial.println(onboardTime);

  // power config: FLASH_TRANSMISSIONS_ENABLED (0), FLASH_LOW_POWER_MODE_ENABLED (1), FLASH_LOW_POWER_MODE (2 - 4), FLASH_MPPT_TEMP_SWITCH_ENABLED (5), FLASH_MPPT_KEEP_ALIVE_ENABLED (6)
  uint8_t powerConfig = random(0, 255);
  memcpy(optDataPtr, &powerConfig, sizeof(powerConfig));
  optDataPtr += sizeof(powerConfig);
  Serial.print("powerConfig: "); Serial.println(powerConfig);

  uint16_t resetCounter = random(0, 100);
  memcpy(optDataPtr, &resetCounter, sizeof(resetCounter));
  optDataPtr += sizeof(resetCounter);
  Serial.print("resetCounter: "); Serial.println(resetCounter);

  uint8_t voltageXA = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &voltageXA, sizeof(voltageXA));
  optDataPtr += sizeof(voltageXA);
  Serial.print("voltageXA: "); Serial.println(voltageXA);

  uint8_t voltageXB = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &voltageXB, sizeof(voltageXB));
  optDataPtr += sizeof(voltageXB);
  Serial.print("voltageXB: "); Serial.println(voltageXB);

  uint8_t voltageZA = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &voltageZA, sizeof(voltageZA));
  optDataPtr += sizeof(voltageZA);
  Serial.print("voltageZA: "); Serial.println(voltageZA);

  uint8_t voltageZB = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &voltageZB, sizeof(voltageZB));
  optDataPtr += sizeof(voltageZB);
  Serial.print("voltageZB: "); Serial.println(voltageZB);

  uint8_t voltageY = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &voltageY, sizeof(voltageY));
  optDataPtr += sizeof(voltageY);
  Serial.print("voltageY: "); Serial.println(voltageY);

  int16_t batteryTemperature = ((float)random(-50000, 120000) / 1000.0) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(optDataPtr, &batteryTemperature, sizeof(batteryTemperature));
  optDataPtr += sizeof(batteryTemperature);
  Serial.print("batteryTemperature: "); Serial.println(batteryTemperature);

  int16_t boardTemperature = ((float)random(-50000, 120000) / 1000.0) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(optDataPtr, &boardTemperature, sizeof(boardTemperature));
  optDataPtr += sizeof(boardTemperature);
  Serial.print("boardTemperature: "); Serial.println(boardTemperature);

  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, RESP_SYSTEM_INFO, optDataLen, optData);
  int state = lora.transmit(frame, len);
  delete[] frame;

  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  Serial.println();

}