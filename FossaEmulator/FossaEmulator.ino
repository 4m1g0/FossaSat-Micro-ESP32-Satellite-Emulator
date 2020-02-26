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
void decodeGS(uint8_t* respFrame, uint8_t respLen);
void decodeSat(uint8_t* optData, size_t optDataLen);

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
    decodeGS(respFrame, respLen);
    decodeSat(respFrame, respLen);
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

void sendStatistics(){
  // TODO: Implement 
  return;

  /*if(Communication_Check_OptDataLen(1, optDataLen)) {

        // response will have maximum of 109 bytes if all stats are included
        uint8_t respOptData[109];
        uint8_t respOptDataLen = 1;
        uint8_t* respOptDataPtr = respOptData;

        // copy stat flags
        uint8_t flags = optData[0];
        memcpy(respOptDataPtr, &flags, sizeof(uint8_t));
        respOptDataPtr += sizeof(uint8_t);

        if(flags & 0b00000001) {
          // temperatures
          PersistentStorage_Read(FLASH_STATS_TEMP_PANEL_Y, respOptDataPtr, 15*sizeof(int16_t));
          respOptDataPtr += 15*sizeof(int16_t);
          respOptDataLen += 15*sizeof(int16_t);
        }

        if(flags & 0b00000010) {
          // currents
          PersistentStorage_Read(FLASH_STATS_CURR_XA, respOptDataPtr, 18*sizeof(int16_t));
          respOptDataPtr += 18*sizeof(int16_t);
          respOptDataLen += 18*sizeof(int16_t);
        }

        if(flags & 0b00000100) {
          // voltages
          PersistentStorage_Read(FLASH_STATS_VOLT_XA, respOptDataPtr, 18*sizeof(uint8_t));
          respOptDataPtr += 18*sizeof(uint8_t);
          respOptDataLen += 18*sizeof(uint8_t);
        }

        if(flags & 0b00001000) {
          // lights
          PersistentStorage_Read(FLASH_STATS_LIGHT_PANEL_Y, respOptDataPtr, 6*sizeof(float));
          respOptDataPtr += 6*sizeof(float);
          respOptDataLen += 6*sizeof(float);
        }

        Communication_Send_Response(RESP_STATISTICS, respOptData, respOptDataLen);
      }*/

}


void sendFullSysInfo() {
  // TODO: Implement 
  return;

}

void decodeGS(uint8_t* respFrame, uint8_t respLen) {
  // print raw data
  Serial.print(F("Received data from the satelite "));
  Serial.print(respLen);
  Serial.println(F(" bytes:"));

  // get function ID
  uint8_t functionId = FCP_Get_FunctionID(callsign, respFrame, respLen);

  if(functionId != RESP_CAMERA_PICTURE) {
    // print packet info
    Serial.print(F("RSSI: "));
    Serial.print(lora.getRSSI());
    Serial.println(F(" dBm"));
    Serial.print(F("SNR: "));
    Serial.print(lora.getSNR());
    Serial.println(F(" dB"));
    Serial.print(F("Function ID: 0x"));
    Serial.println(functionId, HEX);
    PRINT_BUFF(respFrame, respLen);
  }

  // check optional data
  uint8_t* respOptData = nullptr;
  uint8_t respOptDataLen = 0;
  if (functionId < PRIVATE_OFFSET) {
    // public frame
    respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen);
  } else {
    // private frame
    Serial.println("Received private frame. Not supported yet");
    return;
  }
  Serial.print(F("Optional data ("));
  Serial.print(respOptDataLen);
  Serial.println(F(" bytes):"));
  if (respOptDataLen > 0) {
    // read optional data
    respOptData = new uint8_t[respOptDataLen];
    FCP_Get_OptData(callsign, respFrame, respLen, respOptData);
    
    if(functionId != RESP_CAMERA_PICTURE) {
      PRINT_BUFF(respOptData, respOptDataLen);
    }
  }

  // process received frame
  switch (functionId) {
    case RESP_PONG:
      Serial.println(F("Pong!"));
      break;

    case RESP_SYSTEM_INFO: {
      Serial.println(F("System info:"));

      Serial.print(F("batteryVoltage = "));
      Serial.print(FCP_Get_Battery_Voltage(respOptData));
      Serial.println(" V");

      Serial.print(F("batteryChargingCurrent = "));
      Serial.print(FCP_Get_Battery_Charging_Current(respOptData), 4);
      Serial.println(" mA");

      uint32_t onboardTime = 0;
      memcpy(&onboardTime, respOptData + 3, sizeof(uint32_t));
      Serial.print(F("onboardTime = "));
      Serial.println(onboardTime);

      uint8_t powerConfig = 0;
      memcpy(&powerConfig, respOptData + 7, sizeof(uint8_t));
      Serial.print(F("powerConfig = 0b"));
      Serial.println(powerConfig, BIN);

      uint16_t resetCounter = 0;
      memcpy(&resetCounter, respOptData + 8, sizeof(uint16_t));
      Serial.print(F("resetCounter = "));
      Serial.println(resetCounter);

      Serial.print(F("voltageXA = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 10));
      Serial.println(" V");

      Serial.print(F("voltageXB = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 11));
      Serial.println(" V");

      Serial.print(F("voltageZA = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 12));
      Serial.println(" V");

      Serial.print(F("voltageZB = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 13));
      Serial.println(" V");

      Serial.print(F("voltageY = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 14));
      Serial.println(" V");

      Serial.print(F("batteryTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 15));
      Serial.println(" deg C");

      Serial.print(F("boardTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 17));
      Serial.println(" deg C");
      
    } break;

    case RESP_PACKET_INFO: {
      Serial.println(F("Packet info:"));

      Serial.print(F("SNR = "));
      Serial.print(respOptData[0] / 4.0);
      Serial.println(F(" dB"));

      Serial.print(F("RSSI = "));
      Serial.print(respOptData[1] / -2.0);
      Serial.println(F(" dBm"));

      uint16_t counter = 0;
      Serial.print(F("valid LoRa frames = "));
      memcpy(&counter, respOptData + 2, sizeof(uint16_t));
      Serial.println(counter);
      
      Serial.print(F("invalid LoRa frames = "));
      memcpy(&counter, respOptData + 4, sizeof(uint16_t));
      Serial.println(counter);
      
      Serial.print(F("valid FSK frames = "));
      memcpy(&counter, respOptData + 6, sizeof(uint16_t));
      Serial.println(counter);
      
      Serial.print(F("invalid FSK frames = "));
      memcpy(&counter, respOptData + 8, sizeof(uint16_t));
      Serial.println(counter);
    } break;

    case RESP_REPEATED_MESSAGE:
      Serial.println(F("Got repeated message:"));
      for (uint8_t i = 0; i < respOptDataLen; i++) {
        Serial.write(respOptData[i]);
      }
      Serial.println();
      break;

    case RESP_DEPLOYMENT_STATE:
      Serial.println(F("Got deployment counter:"));
      Serial.println(respOptData[0]);
      break;

    case RESP_STATISTICS: {
      Serial.println(F("Got stats:\t\tunit\tmin\tavg\tmax"));
      // TODO stats parsing
    } break;

    case RESP_FULL_SYSTEM_INFO: {
      Serial.println(F("System info:"));

      Serial.print(F("batteryVoltage = "));
      Serial.print(FCP_Get_Battery_Voltage(respOptData));
      Serial.println(" V");

      Serial.print(F("batteryChargingCurrent = "));
      Serial.print(FCP_Get_Battery_Charging_Current(respOptData), 4);
      Serial.println(" mA");

      uint32_t onboardTime = 0;
      memcpy(&onboardTime, respOptData + 3, sizeof(uint32_t));
      Serial.print(F("onboardTime = "));
      Serial.println(onboardTime);

      uint8_t powerConfig = 0;
      memcpy(&powerConfig, respOptData + 7, sizeof(uint8_t));
      Serial.print(F("powerConfig = 0b"));
      Serial.println(powerConfig, BIN);

      uint16_t resetCounter = 0;
      memcpy(&resetCounter, respOptData + 8, sizeof(uint16_t));
      Serial.print(F("resetCounter = "));
      Serial.println(resetCounter);

      Serial.print(F("voltageXA = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 10));
      Serial.println(" V");

      Serial.print(F("currentXA = "));
      Serial.print(FCP_System_Info_Get_Current(respOptData, 11));
      Serial.println(" mA");

      Serial.print(F("voltageXB = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 13));
      Serial.println(" V");

      Serial.print(F("currentXB = "));
      Serial.print(FCP_System_Info_Get_Current(respOptData, 14));
      Serial.println(" mA");

      Serial.print(F("voltageZA = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 16));
      Serial.println(" V");

      Serial.print(F("currentZA = "));
      Serial.print(FCP_System_Info_Get_Current(respOptData, 17));
      Serial.println(" mA");

      Serial.print(F("voltageZB = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 19));
      Serial.println(" V");

      Serial.print(F("currentZB = "));
      Serial.print(FCP_System_Info_Get_Current(respOptData, 20));
      Serial.println(" mA");

      Serial.print(F("voltageY = "));
      Serial.print(FCP_System_Info_Get_Voltage(respOptData, 22));
      Serial.println(" V");

      Serial.print(F("currentY = "));
      Serial.print(FCP_System_Info_Get_Current(respOptData, 23));
      Serial.println(" mA");

      Serial.print(F("tempPanelY = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 25));
      Serial.println(" deg C");

      Serial.print(F("boardTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 27));
      Serial.println(" deg C");

      Serial.print(F("tempBottom = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 29));
      Serial.println(" deg C");

      Serial.print(F("batteryTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 31));
      Serial.println(" deg C");

      Serial.print(F("secBatteryTemp = "));
      Serial.print(FCP_System_Info_Get_Temperature(respOptData, 33));
      Serial.println(" deg C");

      float lightVal = 0;
      memcpy(&lightVal, respOptData + 35, sizeof(float));
      Serial.print(F("lightPanelY = "));
      Serial.println(lightVal, 2);
      
      memcpy(&lightVal, respOptData + 39, sizeof(float));
      Serial.print(F("lightTop = "));
      Serial.println(lightVal, 2);

      uint8_t fault = 0;
      memcpy(&fault, respOptData + 43, sizeof(uint8_t));
      Serial.print(F("faultX = 0x"));
      Serial.println(fault, HEX);
      
      memcpy(&fault, respOptData + 44, sizeof(uint8_t));
      Serial.print(F("faultY = 0x"));
      Serial.println(fault, HEX);
      
      memcpy(&fault, respOptData + 45, sizeof(uint8_t));
      Serial.print(F("faultZ = 0x"));
      Serial.println(fault, HEX);
      
    } break;

    case RESP_CAMERA_PICTURE: {
      uint16_t packetId = 0;
      memcpy(&packetId, respOptData, sizeof(uint16_t));
      Serial.print(F("Packet ID: "));
      Serial.println(packetId);
      
      char buff[16];
      if(respOptDataLen < 16) {
        for(uint8_t i = 0; i < respOptDataLen; i++) {
          sprintf(buff, "%02x ", respOptData[2 + i]);
          Serial.print(buff);
        }
        Serial.println();
      } else {
        for(uint8_t i = 0; i < respOptDataLen/16; i++) {
          for(uint8_t j = 0; j < 16; j++) {
            sprintf(buff, "%02x ", respOptData[2 + i*16 + j]);
            Serial.print(buff);
          }
          Serial.println();
        }
      }
    } break;

    default:
      break;
  }
  
  if (respOptDataLen > 0) {
    delete[] respOptData;
  }
}



void decodeSat(uint8_t* optData, size_t optDataLen) {

  Serial.print(F("Received data from a ground station. "));
  Serial.print(optDataLen);
  Serial.println(F("bytes: "));

  uint8_t functionId = FCP_Get_FunctionID(callsign, optData, optDataLen);

  // execute function based on ID
  switch (functionId) {

    // public function IDs

    case CMD_PING: {
      // send pong
      Communication_Send_Response(RESP_PONG);
    } break;

    case CMD_RETRANSMIT: {
        // check message length
        if (optDataLen <= MAX_STRING_LENGTH) {
          // respond with the requested data
          Communication_Send_Response(RESP_REPEATED_MESSAGE, optData, optDataLen);
        }
      } break;

    case CMD_RETRANSMIT_CUSTOM: {
        // check message length
        if ((optDataLen >= 8) && (optDataLen <= MAX_STRING_LENGTH + 7)) {
          // check bandwidth value (loaded from array - rest of settings are checked by library)
          if (optData[0] > 7) {
            Serial.print(F("Invalid BW "));
            Serial.println(optData[0]);
            break;
          }

          // attempt to change the settings
          float bws[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0};
          uint16_t preambleLength = 0;
          memcpy(&preambleLength, optData + 3, sizeof(uint16_t));

          // change modem configuration
          Serial.println(F("Asked to retransmit with this configuration: "));
          Serial.print("BW: "); Serial.println(bws[optData[0]]);
          Serial.print("SF: "); Serial.println(optData[1]);
          Serial.print("CR: "); Serial.println(optData[2]);
          Serial.print("PreambleLen: "); Serial.println(preambleLength);
          Serial.print("CRC: "); Serial.println(optData[5]?"true":"false");
          Serial.print("Power: "); Serial.println(optData[6]);

          // configuration changed successfully, transmit response
          Communication_Send_Response(RESP_REPEATED_MESSAGE_CUSTOM, optData + 7, optDataLen - 7, true);
        }
      } break;

    case CMD_TRANSMIT_SYSTEM_INFO: {
      // send system info via LoRa
      sendSysInfo();
    } break;

    case CMD_GET_PACKET_INFO: {
      sendPacketInfo();
    } break;

    case CMD_GET_STATISTICS: {
      sendStatistics();
    } break;

    case CMD_GET_FULL_SYSTEM_INFO: {
      sendFullSysInfo();
    } break;

    case CMD_STORE_AND_FORWARD_ADD: {
      // TODO:
    } break;

    case CMD_STORE_AND_FORWARD_REQUEST: {
      // TODO:
    } break;

    // private frames below this line
    case CMD_DEPLOY: 
    case CMD_RESTART: 
    case CMD_SET_TRANSMIT_ENABLE: 
    case CMD_SET_CALLSIGN: 
    case CMD_SET_SF_MODE: 
    case CMD_SET_LOW_POWER_ENABLE: 
    case CMD_SET_MPPT_MODE:
    case CMD_SET_RECEIVE_WINDOWS: 
    case CMD_CAMERA_CAPTURE:
    case CMD_SET_POWER_LIMITS: 
    case CMD_SET_RTC: 
    case CMD_RECORD_IMU: 
    case CMD_RUN_ADCS: 
    case CMD_GET_PICTURE_BURST: 
    case CMD_GET_FLASH_CONTENTS: 
    case CMD_GET_PICTURE_LENGTH: 
    case CMD_LOG_GPS:
    case CMD_GET_GPS_LOG: 
    case CMD_ROUTE:
        Serial.println(F("Received private command. Not supported yet."));
        break;
    default:
      return;
  }
}