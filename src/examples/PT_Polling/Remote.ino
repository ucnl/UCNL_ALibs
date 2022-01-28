/*
  Copyright (C) 2021, Underwater communication & navigation laboratory
  All rights reserved.

  www.unavlab.com
  hello@unavlab.com

*/
/* uWave modem connected to a Arduino Nano as follows:

    uWave TX -> UWAVE_TX_SW_PIN
    uWave RX -> UWAVE_RX_SW_PIN
    uWave CMD/SVC -> UWAVE_CMD_PIN

   The sketch:

   Sets the proper settings of Tx/Rx channel IDs, proper PT mode address and waits for an
   incoming 1-byte packet. If the packet contains own PT mode address, the 5-byte
   packet is built: 0 - data type - means tank pressure, bytes from 1 to 4 filled with 32-bit float value
   that is taken from a ADC pin.
*/

#include "ucnl_str.h"
#include "ucnl_nmea.h"
#include "ucnl_uwave.h"
#include "ucnl_wphx.h"

#include "SoftwareSerial.h"

#define USE_SERIAL_OUT                    // Comment this define to disable output to Serial (USB on Arduino board)

#define UWAVE_TX_SW_PIN      (3)
#define UWAVE_RX_SW_PIN      (2)
#define UWAVE_CMD_PIN        (4)

SoftwareSerial uPort(UWAVE_TX_SW_PIN, UWAVE_RX_SW_PIN);

#define REM_TX_ID            (0)          // Remote's tx channel ID
#define REM_RX_ID            (0)          // Remote's rx channel ID

#define BASE_PT_ADDR         (0)          // Base station (remote) packet mode address
#define OWN_PT_ADDR          (1)          // This device packet mode address
#define UPLINK_PT_TRIES      (4)          // Number of tries for sending data in the packet mode

#define OWN_TX_ID            (REM_RX_ID)  // Own tx channel ID
#define OWN_RX_ID            (REM_TX_ID)  // Own rx channel ID

#define ANALOG_PIN           (A3)         // ADC pin to get the value to be sent

#define UART_IN_BUFFER_SIZE  (127)
#define UART_OUT_BUFFER_SIZE (127)

#define LOC_TIMEOUT_MS       (5000)       // Local device timeout

#define WATER_SALINITY_PSU   (0)          // Water salinity

byte                          uwave_in_buffer[UART_IN_BUFFER_SIZE];
byte                          uwave_out_buffer[UART_OUT_BUFFER_SIZE];
byte                          uwave_out_buffer_idx;

UCNL_NMEA_Result_Enum         parserResult;
UCNL_NMEA_State_Struct        uwaveParser;

uWAVE_ACK_RESULT_Struct       ackData;
uWAVE_DINFO_Struct            dinfoData;
uWAVE_SETTINGS_WRITE_Struct   settingsData;
uWAVE_PT_SETTINGS_Struct      ptSettingsData;
uWAVE_PT_PACKET_Struct        ptPacketData;


#define uWAVE_SNT_IDS_SIZE    (6)
long                          uwaveSntIDs[] = { uWAVE_NMEA_UWV0_SNT_ID,     // ACK
                                                uWAVE_NMEA_UWVE_SNT_ID,     // PT_SETTINGS
                                                uWAVE_NMEA_UWVH_SNT_ID,     // PT_FAILED
                                                uWAVE_NMEA_UWVI_SNT_ID,     // PT_DLVRD
                                                uWAVE_NMEA_UWVJ_SNT_ID,     // PT_RCVD
                                                uWAVE_NMEA_UWV_EXCL_SNT_ID, // DINFO
                                              };

long                          ts;
long                          loc_req_ts          = 0;
bool                          is_loc_waiting      = false;
bool                          dinfo_queried       = false;
bool                          settings_updated    = false;
bool                          pt_settings_queried = false;
bool                          pt_settings_updated = false;

byte                          ptPacket[uWAVE_PKT_MAX_SIZE];

#define DUMMY_SIZE            (32)
byte                          serialNumber[DUMMY_SIZE];
byte                          coreMoniker[DUMMY_SIZE];
byte                          sysMoniker[DUMMY_SIZE];


//
void C_OnRequestBuilt() {
  uPort.write(uwave_out_buffer, uwave_out_buffer_idx);
  loc_req_ts = millis();
  is_loc_waiting = true;
}

void setup () {

  delay(100);
  pinMode(UWAVE_CMD_PIN, OUTPUT);
  digitalWrite(UWAVE_CMD_PIN, HIGH); // uWave CMD mode is enabled
  delay(200);

#ifdef USE_SERIAL_OUT
  Serial.begin(9600);
#endif

  uPort.begin(9600);

  settingsData.rxChID            = OWN_RX_ID;
  settingsData.txChID            = OWN_TX_ID;
  settingsData.styPSU            = WATER_SALINITY_PSU;
  settingsData.isCmdMode         = false;
  settingsData.isACKOnTXFinished = false;
  settingsData.gravityAcc        = UCNL_WPHX_GRAVITY_ACC_MPS2;

  ptPacketData.dataPacket        = ptPacket;

  dinfoData.serialNumber = serialNumber;
  dinfoData.sys_moniker  = sysMoniker;
  dinfoData.core_moniker = coreMoniker;

  UCNL_NMEA_InitStruct(&uwaveParser, uwave_in_buffer, UART_IN_BUFFER_SIZE, uwaveSntIDs, uWAVE_SNT_IDS_SIZE);

#ifdef USE_SERIAL_OUT
  Serial.println(F("Hello from UC&NL!"));
#endif
}

void loop () {

  if (uPort.available()) {
    
    byte b = uPort.read();
    parserResult = UCNL_NMEA_Process_Byte(&uwaveParser, b);
    
    if (parserResult == UCNL_NMEA_RESULT_PACKET_READY) {

      is_loc_waiting = false;

      // IC_D2H_ACK
      if ((uwaveParser.sntID == uWAVE_NMEA_UWV0_SNT_ID) &&
          (uWAVE_Parse_ACK(&ackData, uwaveParser.buffer, uwaveParser.idx))) {

        if (ackData.sentenceID == IC_H2D_SETTINGS_WRITE) {
          if (ackData.errCode == LOC_ERR_NO_ERROR) {

            settings_updated = true;
#ifdef USE_SERIAL_OUT
            Serial.println(F("Device settings updated"));
#endif
          }
        }
        else if (ackData.sentenceID == IC_H2H_PT_SETTINGS_WRITE) {
          if (ackData.errCode == LOC_ERR_NO_ERROR) {

            pt_settings_updated = true;
#ifdef USE_SERIAL_OUT
            Serial.println(F("Packet mode settings updated"));
#endif
          }
        }
        else if (ackData.sentenceID == IC_H2D_PT_SEND) {
          if (ackData.errCode == LOC_ERR_NO_ERROR) {

#ifdef USE_SERIAL_OUT
            Serial.println(F("Data packet has been sent"));
#endif
          }
          else {
#ifdef USE_SERIAL_OUT
            Serial.print(F("Packet send request was not accepted: "));
            Serial.println(ackData.errCode);
#endif
          }
        }
      }

      // IC_D2H_DINFO
      if ((uwaveParser.sntID == uWAVE_NMEA_UWV_EXCL_SNT_ID) &&
          (uWAVE_Parse_DINFO(&dinfoData, uwaveParser.buffer, uwaveParser.idx))) {
        if ((dinfoData.rxChID == OWN_RX_ID) &&
            (dinfoData.txChID == OWN_TX_ID) &&
            (dinfoData.styPSU == WATER_SALINITY_PSU) &&
            (dinfoData.isCmdMode == false)) {

          settings_updated = true;

#ifdef USE_SERIAL_OUT
          Serial.println(F("Device settings is relevant"));
#endif
        }

        dinfo_queried = true;
      }

      // IC_D2H_PT_SETTINGS
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVE_SNT_ID) &&
               (uWAVE_Parse_PT_SETTINGS(&ptSettingsData, uwaveParser.buffer, uwaveParser.idx))) {
        if ((ptSettingsData.isPtEnabled) &&
            (ptSettingsData.ptAddress == OWN_PT_ADDR)) {
              
          pt_settings_updated = true;

#ifdef USE_SERIAL_OUT
          Serial.println(F("Packet mode settings is relevant"));
#endif
        }

        pt_settings_queried = true;
      }

      // IC_D2H_PT_RCVD
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVJ_SNT_ID) &&
               (uWAVE_Parse_PT_RCVD(&ptPacketData, uwaveParser.buffer, uwaveParser.idx))) {

#ifdef USE_SERIAL_OUT
          Serial.print(F("Received a packet from #"));
          Serial.print(ptPacketData.ptAddress);
          Serial.print(F(" Packet size: "));
          Serial.println(ptPacketData.dataPacketSize);
#endif

        if ((ptPacketData.dataPacketSize == 1) &&
            (ptPacketData.dataPacket[0] == OWN_PT_ADDR))
        {          
          ptPacketData.dataPacket[0] = 0; // let the first byte will be a data type

          union u_tag {
            byte b[4];
            float fval;
          } u;

          u.fval = analogRead(ANALOG_PIN); // let read the value to transmit from the specified ADC pin
          // F32 is a kind of an overkill for 10-bit adc readings, but
          // one can easily change the message format

#ifdef USE_SERIAL_OUT
          Serial.print(F("Sending back a f32 value: "));
          Serial.println(u.fval, 3);          
#endif

          ptPacketData.dataPacket[1]  = u.b[0];
          ptPacketData.dataPacket[2]  = u.b[1];
          ptPacketData.dataPacket[3]  = u.b[2];
          ptPacketData.dataPacket[4]  = u.b[3];
          ptPacketData.dataPacketSize = 5;

          ptPacketData.ptAddress      = BASE_PT_ADDR;
          ptPacketData.tries          = UPLINK_PT_TRIES;

          uWAVE_Build_PT_SEND(&ptPacketData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
          C_OnRequestBuilt();
        }
      }

      // IC_D2H_PT_DLVRD
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVI_SNT_ID) &&
               (uWAVE_Parse_PT_DLVRD(&ptPacketData, uwaveParser.buffer, uwaveParser.idx))) {

#ifdef USE_SERIAL_OUT
          Serial.println(F("Packet has been delivered :-)"));
#endif
      }

      // IC_D2H_PT_FAILED
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVH_SNT_ID) &&
               (uWAVE_Parse_PT_FAILED(&ptPacketData, uwaveParser.buffer, uwaveParser.idx))) {
                
#ifdef USE_SERIAL_OUT
          Serial.println(F("Packet delivery is failed :-\\"));
#endif
                
      }
      
    }
    UCNL_NMEA_Release(&uwaveParser);
  }

  ts = millis();
  if ((is_loc_waiting) &&
      (ts - loc_req_ts >= LOC_TIMEOUT_MS)) {

    is_loc_waiting = false;
#ifdef USE_SERIAL_OUT
          Serial.println(F("Local timeout"));
#endif
  }

  if (!is_loc_waiting) {

    if (!dinfo_queried) {
      uWAVE_Build_DINFO_GET(uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();
    }

    else if (!settings_updated) {
      uWAVE_Build_SETTINGS_WRITE(&settingsData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();
    }

    else if (!pt_settings_queried) {
      uWAVE_Build_PT_SETTINGS_READ(uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();
    }

    else if (!pt_settings_updated) {

      ptSettingsData.isSaveInFlash   = true;
      ptSettingsData.isPtEnabled     = true; // not necessary since 1.20
      ptSettingsData.ptAddress       = OWN_PT_ADDR;
      uWAVE_Build_PT_SETTINGS_WRITE(&ptSettingsData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();
    }
  }
}
