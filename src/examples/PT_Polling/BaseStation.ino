/*
  Copyright (C) 2021, Underwater communication & navigation laboratory
  All rights reserved.

  www.unavlab.com
  hello@unavlab.com

*/

/* uWave modem connected to a Arduino MEGA as follows:

    uWave TX -> Serial 1 RX (pin 19)
    uWave RX -> Serial 1 TX (pin 18)
    uWave CMD/SVC -> see the #define UWAVE_CMD_PIN

   The sketch:

   1. Queries the device information, compares to a specified Tx and Rx channels IDs OWN_RX_ID and OWN_TX_ID
      if settings differ, tries to apply new settings
   2. Queries the device's packet mode settings, compares to a specified OWN_PT_ADDR and tries to apply new
      packet mode settings
   3. Polls remotes in packet mode from REM_PT_ADDR_FROM to REM_PT_ADDR_TO. Every poll goes in two stages:
      The first stage is done by PT_ITG request (command requests equivalent for packet mode). If the request
      causes a timeout, the sketch switches to the next remote. If the request is succeeded, the sketch
      sends a 1-byte broadcast request that contains a remotes address. The requested remote should send
      a 5-byte packet as a response: 1st byte is a "data type", next 4 bytes
      is a 32-bit float

   All information translates to arduino serial (0).
*/

#include "ucnl_str.h"
#include "ucnl_nmea.h"
#include "ucnl_uwave.h"
#include "ucnl_wphx.h"

#define USE_SERIAL_OUT                   // Comment this define to disable output to Serial (USB on Arduino board)

#define UWAVE_CMD_PIN        (15)        // Pin on the arduino board to connect the SVC/CMD wire

#define REM_TX_ID            (0)         // Remote's transmitter channel ID
#define REM_RX_ID            (0)         // Remote's receiver channel ID

#define OWN_PT_ADDR          (0)         // Packet mode address for the local modem
#define REM_PT_ADDR_FROM     (1)         // Packet mode remotes addresses starts from
#define REM_PT_ADDR_TO       (2)         // Packet mode remotes addresses ends at

#define OWN_TX_ID            (REM_RX_ID) // Own transmitter channel ID
#define OWN_RX_ID            (REM_TX_ID) // Own receiver channel ID

#define WATER_SALINITY_PSU   (0)         // Water salinity, PSU

#define LOC_TIMEOUT_MS       (5000)      // Local query timeout
#define REM_TIMEOUT_MS       (20000)     // Remote response timeout for incoming data packet
#define REM_TIMEOUT_ITG_MS   (4000)      // Remote response timeout for ITG requests

#define REMOTE_TIMESLICE_MS  (5000)      // minimal time gap between querying different remotes

#define UART_IN_BUFFER_SIZE  (127)
#define UART_OUT_BUFFER_SIZE (127)

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
uWAVE_PT_ITG_Struct           ptITGData;
uWAVE_PT_ITG_RESP_Struct      ptITGRespData;

#define uWAVE_SNT_IDS_SIZE    (7)
long                          uwaveSntIDs[] = { uWAVE_NMEA_UWV0_SNT_ID,     // ACK
                                                uWAVE_NMEA_UWVE_SNT_ID,     // PT_SETTINGS
                                                uWAVE_NMEA_UWVH_SNT_ID,     // PT_FAILED
                                                uWAVE_NMEA_UWVJ_SNT_ID,     // PT_RCVD
                                                uWAVE_NMEA_UWVL_SNT_ID,     // PT_TMO
                                                uWAVE_NMEA_UWVM_SNT_ID,     // PT_ITG_RESP
                                                uWAVE_NMEA_UWV_EXCL_SNT_ID, // DINFO
                                              };

// Speed of sound in water
float                        sound_speed_mps = UCNL_WPHX_FWTR_SOUND_SPEED_MPS;

// system's state machine variabled
int                          rem_timeout_ms = REM_TIMEOUT_MS;

long                         ts;
long                         loc_req_ts = 0;
bool                         is_loc_waiting = false;

long                         rem_req_ts = 0;
bool                         is_rem_waiting = false;
bool                         dinfo_queried = false;
bool                         settings_updated = false;
bool                         pt_settings_queried = false;
bool                         pt_settings_updated = false;
bool                         request_stage_two = false;
byte                         target_pt_addr = REM_PT_ADDR_FROM;

byte                         ptPacket[uWAVE_PKT_MAX_SIZE];

#define                      DUMMY_SIZE      (32)
byte                         serialNumber[DUMMY_SIZE];
byte                         coreMoniker[DUMMY_SIZE];
byte                         sysMoniker[DUMMY_SIZE];

//
void C_OnRequestBuilt() {
  Serial1.write(uwave_out_buffer, uwave_out_buffer_idx);
  loc_req_ts = millis();
  is_loc_waiting = true;
}

void C_NextRemote() {

  request_stage_two = false;
  target_pt_addr++;

  if (target_pt_addr > REM_PT_ADDR_TO)
    target_pt_addr = REM_PT_ADDR_FROM;
}

void C_ProcessRemoteRequests() {

  // Every remote will be queried twice: at the first stage it is queried
  // with ITG request that allows getting the propagation time (and the slant range) and some remote's
  // telemetry data - depth, water temperature, and supply voltage
  // at the second stage we send a remote's packet mode address to the broadcast address
  // The remote with the specified address, once the packet is received, will transfer the data to its
  // control system (another Arduino e.g.). The control system will check if it is its address and if so
  // will send a 5-byte packet to the base station: 1st byte is a data type marker, other 4 bytes is a 32-bit float value

  if (request_stage_two) {

    ptPacketData.ptAddress      = uWAVE_PKT_BCAST_ADDR; // Broadcasting message
    ptPacketData.tries          = 1;                    // does not matter for broadcast messages

    ptPacketData.dataPacket[0]  = target_pt_addr;       // Broadcasting just a remote's address as a 1-byte packet
    ptPacketData.dataPacketSize = 1;                    //

    uWAVE_Build_PT_SEND(&ptPacketData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);

#ifdef USE_SERIAL_OUT
    Serial.print("Stage 2 Querying remote #");
    Serial.println(target_pt_addr);
#endif

    C_OnRequestBuilt();
  }
  else if (ts - rem_req_ts >= REMOTE_TIMESLICE_MS) {  // To make requests less frequent just to save the batteries

    ptITGData.ptAddress = target_pt_addr;
    ptITGData.pt_itg_dataID = (ptITGData.pt_itg_dataID + 1) % DID_INVALID;
    uWAVE_Build_PT_ITG(&ptITGData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);

#ifdef USE_SERIAL_OUT
    Serial.print("Stage 1 Querying remote #");
    Serial.println(target_pt_addr);
#endif

    C_OnRequestBuilt();
  }
}

void setup () {

  delay(100);
  pinMode(UWAVE_CMD_PIN, OUTPUT);
  digitalWrite(UWAVE_CMD_PIN, HIGH); // uWave CMD mode is enabled
  delay(200);

#ifdef USE_SERIAL_OUT
  Serial.begin(9600);
#endif

  Serial1.begin(9600);

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
  Serial.println("Hello from UC&NL!");
#endif
}

void loop () {
  if (Serial1.available()) {
    byte b = Serial1.read();
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
            Serial.println("Device settings updated");
#endif
          }
        }
        else if (ackData.sentenceID == IC_H2H_PT_SETTINGS_WRITE) {
          if (ackData.errCode == LOC_ERR_NO_ERROR) {
            pt_settings_updated = true;

#ifdef USE_SERIAL_OUT
            Serial.println("Packet mode settings updated");
#endif

          }
        }
        else if (ackData.sentenceID == IC_H2D_PT_SEND) {
          if (ackData.errCode == LOC_ERR_NO_ERROR) {
            is_rem_waiting = true;
            rem_timeout_ms = REM_TIMEOUT_MS;
            rem_req_ts = millis();

#ifdef USE_SERIAL_OUT
            Serial.println("Remote request stage 2 accepted");
#endif
          }
          else
          {
#ifdef USE_SERIAL_OUT
            Serial.print("Remote request is not accepted: ");
            Serial.println(ackData.errCode);
#endif
          }
        }
        else if (ackData.sentenceID == IC_H2D_PT_ITG) {
          if (ackData.errCode == LOC_ERR_NO_ERROR) {
            is_rem_waiting = true;
            rem_timeout_ms = REM_TIMEOUT_ITG_MS;
            rem_req_ts = millis();
#ifdef USE_SERIAL_OUT
            Serial.println("Remote request stage 1 accepted");
#endif
          }
          else
          {
#ifdef USE_SERIAL_OUT
            Serial.print("Remote request is not accepted: ");
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
          Serial.println("Device settings is relevant");
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
          Serial.println("Packet mode settings is relevant");
#endif
        }
      }

      // IC_D2H_PT_RCVD
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVJ_SNT_ID) &&
               (uWAVE_Parse_PT_RCVD(&ptPacketData, uwaveParser.buffer, uwaveParser.idx))) {

        is_rem_waiting = false;

        if (ptPacketData.dataPacketSize == 5)
        {
          byte dataID = ptPacketData.dataPacket[0]; // a Data ID

          union u_tag {
            byte b[4];
            float fval;
          } u;

          u.b[0] = ptPacketData.dataPacket[1];
          u.b[1] = ptPacketData.dataPacket[2];
          u.b[2] = ptPacketData.dataPacket[3];
          u.b[3] = ptPacketData.dataPacket[4];

          float dataValue = u.fval; // a data value - 32-bit float in our case

#ifdef USE_SERIAL_OUT
          Serial.print("Remote #");
          Serial.print(ptPacketData.ptAddress);
          Serial.print(" Value: ");
          Serial.println(dataValue, 3);
#endif

          C_NextRemote();
        }
      }

      // IC_D2H_PT_TMO
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVL_SNT_ID) &&
               (uWAVE_Parse_PT_TMO(&ptITGData, uwaveParser.buffer, uwaveParser.idx))) {

        is_rem_waiting = false;

#ifdef USE_SERIAL_OUT
        Serial.print("Remote device #");
        Serial.print(ptITGData.ptAddress);
        Serial.print(" timeout (");
        Serial.print(ptITGData.pt_itg_dataID);
        Serial.println(")");
#endif

        C_NextRemote();
      }

      // IC_D2H_PT_ITG_RESP
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWVM_SNT_ID) &&
               (uWAVE_Parse_PT_ITG_RESP(&ptITGRespData, uwaveParser.buffer, uwaveParser.idx))) {

        is_rem_waiting = false;
        request_stage_two = true;

#ifdef USE_SERIAL_OUT
        Serial.print("Remote device #");  Serial.println(ptITGRespData.target_ptAddress);
        Serial.print("pTime, sec: ");     Serial.println(ptITGRespData.pTime, 5);
        Serial.print("Slant range, m: "); Serial.println(ptITGRespData.pTime * sound_speed_mps);

        if (ptITGRespData.isValue) {
          if (ptITGRespData.pt_itg_dataID == DID_DPT) {
            Serial.print("Depth, m: ");
          }
          else if (ptITGRespData.pt_itg_dataID == DID_TMP) {

            sound_speed_mps = UCNL_WPHX_speed_of_sound_UNESCO_calc(ptITGRespData.dataValue, UCNL_WPHX_ATM_PRESSURE_MBAR, WATER_SALINITY_PSU);
            Serial.print("SOS, m/s: ");
            Serial.println(sound_speed_mps, 1);

            Serial.print("Water temperature, °C: ");
          }
          else if (ptITGRespData.pt_itg_dataID == DID_BAT) {
            Serial.print("Supply voltage, V: ");
          }

          Serial.println(ptITGRespData.dataValue);
        }
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
    Serial.println("Local device timeout");
#endif
  }

  if ((is_rem_waiting) &&
      (ts - rem_req_ts >= rem_timeout_ms)) {

    is_rem_waiting = false;
    C_NextRemote();


#ifdef USE_SERIAL_OUT
    Serial.print("Remote device #");
    Serial.print(target_pt_addr);
    Serial.println(" timeout");
#endif
  }

  if (!is_loc_waiting) {

    if (!dinfo_queried) {
      uWAVE_Build_DINFO_GET(uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();

#ifdef USE_SERIAL_OUT
      Serial.println("Querying local device info...");
#endif
    }

    else if (!settings_updated) {
      uWAVE_Build_SETTINGS_WRITE(&settingsData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();

#ifdef USE_SERIAL_OUT
      Serial.println("Updating device settings...");
#endif
    }

    else if (!pt_settings_queried) {

      uWAVE_Build_PT_SETTINGS_READ(uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();
      pt_settings_queried = true;

#ifdef USE_SERIAL_OUT
      Serial.println("Querying packet mode settings...");
#endif
    }

    else if (!pt_settings_updated) {

      ptSettingsData.isSaveInFlash   = true;
      ptSettingsData.isPtEnabled     = true; // not necessary since 1.24
      ptSettingsData.ptAddress       = OWN_PT_ADDR;
      uWAVE_Build_PT_SETTINGS_WRITE(&ptSettingsData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
      C_OnRequestBuilt();

#ifdef USE_SERIAL_OUT
      Serial.println("Updating packet mode settings...");
#endif
    }

    else if (!is_rem_waiting) {
      C_ProcessRemoteRequests();
    }
  }
}
