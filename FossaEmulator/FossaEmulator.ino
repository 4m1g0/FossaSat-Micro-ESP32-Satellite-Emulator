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
void sendSysInfo(bool malformed = false);

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
  //lora.setDio1Action(setFlag);
  lora.setDio0Action(setFlag);
  
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

  Serial.println();
}

void sendSysInfo(bool malformed) {
  size_t optDataLen = 6*sizeof(uint8_t) + 3*sizeof(int16_t) + sizeof(uint16_t) + sizeof(int8_t);
  uint8_t* optData = new uint8_t[optDataLen];
  uint8_t* optDataPtr = optData;

  Serial.println(F("System info:"));

  // set batteryChargingVoltage variable
  uint8_t batteryChargingVoltage = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &batteryChargingVoltage, sizeof(uint8_t));
  optDataPtr += sizeof(uint8_t);
  Serial.print(batteryChargingVoltage);
  Serial.print('*');
  Serial.print(VOLTAGE_MULTIPLIER);
  Serial.println(F(" mV"));

  // set batteryChragingCurrent variable
  int16_t batteryChragingCurrent = ((float)random(-50000, 50000) / 1000000.0) * (CURRENT_UNIT / CURRENT_MULTIPLIER);
  memcpy(optDataPtr, &batteryChragingCurrent, sizeof(int16_t));
  optDataPtr += sizeof(int16_t);
  Serial.print(batteryChragingCurrent);
  Serial.print('*');
  Serial.print(CURRENT_MULTIPLIER);
  Serial.println(F(" uA"));

  // set batteryVoltage variable
  uint8_t batteryVoltage = ((float)random(1800, 3600) / 1000.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &batteryVoltage, sizeof(uint8_t));
  optDataPtr += sizeof(uint8_t);
  Serial.print(batteryVoltage);
  Serial.print('*');
  Serial.print(VOLTAGE_MULTIPLIER);
  Serial.println(F(" mV"));

  // set solarCellAVoltage variable
  uint8_t solarCellAVoltage = ((float)random(0, 330) / 100.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &solarCellAVoltage, sizeof(uint8_t));
  optDataPtr += sizeof(uint8_t);
  Serial.print(solarCellAVoltage);
  Serial.print('*');
  Serial.print(VOLTAGE_MULTIPLIER);
  Serial.println(F(" mV"));

  // set solarCellBVoltage variable
  uint8_t solarCellBVoltage = ((float)random(0, 330) / 100.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &solarCellBVoltage, sizeof(uint8_t));
  optDataPtr += sizeof(uint8_t);
  Serial.print(solarCellBVoltage);
  Serial.print('*');
  Serial.print(VOLTAGE_MULTIPLIER);
  Serial.println(F(" mV"));

  // set solarCellCVoltage variable
  uint8_t solarCellCVoltage = ((float)random(0, 330) / 100.0) * (VOLTAGE_UNIT / VOLTAGE_MULTIPLIER);
  memcpy(optDataPtr, &solarCellCVoltage, sizeof(uint8_t));
  optDataPtr += sizeof(uint8_t);
  Serial.print(solarCellCVoltage);
  Serial.print('*');
  Serial.print(VOLTAGE_MULTIPLIER);
  Serial.println(F(" mV"));

  // set batteryTemperature variable
  int16_t batteryTemperature = ((float)random(-50000, 120000) / 1000.0) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(optDataPtr, &batteryTemperature, sizeof(int16_t));
  optDataPtr += sizeof(int16_t);
  Serial.print(batteryTemperature);
  Serial.print('*');
  Serial.print(TEMPERATURE_MULTIPLIER);
  Serial.println(F(" mdeg C"));

  // set boardTemperature variable
  int16_t boardTemperature = ((float)random(-50000, 120000) / 1000.0) * (TEMPERATURE_UNIT / TEMPERATURE_MULTIPLIER);
  memcpy(optDataPtr, &boardTemperature, sizeof(int16_t));
  optDataPtr += sizeof(int16_t);
  Serial.print(boardTemperature);
  Serial.print('*');
  Serial.print(TEMPERATURE_MULTIPLIER);
  Serial.println(F(" mdeg C"));

  // set mcuTemperature variable (read twice since first value is often nonsense)
  int8_t mcuTemperature = random(0,300) - 150;
  memcpy(optDataPtr, &mcuTemperature, sizeof(int8_t));
  optDataPtr += sizeof(int8_t);
  Serial.println(mcuTemperature);

  // set resetCounter variable
  uint16_t resetCounter = 3;
  memcpy(optDataPtr, &resetCounter, sizeof(uint16_t));
  optDataPtr += sizeof(uint16_t);
  Serial.println(resetCounter);

  // set powerConfig variable
  uint8_t powerConfig = 0xFF;
  Serial.print(F("Config: 0b"));
  Serial.println(powerConfig, BIN);
  memcpy(optDataPtr, &powerConfig, sizeof(uint8_t));
  optDataPtr += sizeof(uint8_t);

  Serial.print(F("Sending sysInfo frame ... "));
  uint8_t functionId = RESP_SYSTEM_INFO;
  if (malformed)
    functionId = 0xF3; // malformed

  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, optDataPtr);
  int state = lora.transmit(frame, len);
  delete[] frame;

  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  Serial.println();

  // deallocate memory
  delete[] optData;
}