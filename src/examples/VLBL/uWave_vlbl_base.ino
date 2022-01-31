#include "ucnl_str.h"
#include "ucnl_nmea.h"
#include "ucnl_uwave.h"
#include "ucnl_wphx.h"
#include "ucnl_nav.h"
#include "ucnl_vlbl.h"

#define USE_LCD
#define USE_SERIAL_OUT

#ifdef USE_LCD

#define BUTTON_PIN      (6)
#define BTN_ACTIVE      (HIGH)
#define BTN_DEBOUNCE_MS (300)
#define SCREEN_NUM      (3)

byte screenID = 0;

/* Screen #0
   |DST ---m |AZM ---°   |
   |DRMS --m |CRS ---°   |
   |GNSS WAIT|DPT ---m   |
   |MOVE ---m|TMO ---    |

   Screen #1
   |GLAT:  --.------° N  |
   |GLON: ---.------° E  |
   |RLAT:  --.------° N  |
   |RLON: ---.------° E  |

   Screen #2
   |SOS : 1450.0 m/s     |
   |WTMP(R): ---.- °C    |
   |WTMP(S): ---.- °C    |
   |BAT R/S: --/-- V     |

*/

#include "LiquidCrystal.h"
#define LCD_RS (12)
#define LCD_EN (11)
#define LCD_D4 (5)
#define LCD_D5 (4)
#define LCD_D6 (3)
#define LCD_D7 (2)
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define LCD_STR_WIDTH (20)

long btn_check_ts = 0;

#endif

#define FW_MONIKER "uWAVE VLBL Ino\0"
#define FW_VERSION 0x0100

#define FW_VERSION_MAJOR (FW_VERSION >> 8)
#define FW_VERSION_MINOR (FW_VERSION & 0xFF)

#define UWAVE_CMD_PIN (15)

#define REMOTE_TX_ID (0)
#define REMOTE_RX_ID (0)

#define LOC_TIMEOUT_MS     (5000)
#define MIN_BASE_SIZE_M    (20)
#define BASE_SIZE_FACTOR   (0.3)
#define WATER_SALINITY_PSU (0)
#define DRMS_THRESHOLD_M   (5)

#define UART_IN_BUFFER_SIZE  (127)
#define UART_OUT_BUFFER_SIZE (64)

byte gnss_in_buffer[UART_IN_BUFFER_SIZE];
byte uwave_in_buffer[UART_IN_BUFFER_SIZE];
byte uwave_out_buffer[UART_OUT_BUFFER_SIZE];
byte uwave_out_buffer_idx;

UCNL_NMEA_State_Struct gnssParser;
UCNL_NMEA_Result_Enum  parserResult;
UCNL_NMEA_RMC_RESULT_Struct gnssRMCData;

#define GNSS_DATA_VALIDITY_HYST (20)
#define GNSS_SNT_IDS_SIZE (1)
long gnssSntIDs[] = { UCNL_NMEA_RMC_SNT_ID };

UCNL_NMEA_State_Struct   uwaveParser;
uWAVE_ACK_RESULT_Struct  ackData;
uWAVE_RC_RESPONSE_Struct rcResponseData;
uWAVE_RC_TIMEOUT_Struct  rcTimeoutData;
uWAVE_AMB_DTA_Struct     ambData;
uWAVE_RC_REQUEST_Struct  rcRequestData;

#define uWAVE_AMB_DTA_CFG_SNT "$PUWV6,1,1,1,1,1,1*32\r\n\0"

#define uWAVE_SNT_IDS_SIZE (4)
long uwaveSntIDs[] = { uWAVE_NMEA_UWV0_SNT_ID, uWAVE_NMEA_UWV3_SNT_ID, uWAVE_NMEA_UWV4_SNT_ID, uWAVE_NMEA_UWV7_SNT_ID };

#define INVALID_FLOAT  (-32768)
#define IS_F_IV(value) ((value) == INVALID_FLOAT)

VLBL_Points_Ring_Struct pointsRing;
VLBL_Ring_Struct        heapsRing;

// Speed of sound in water
float sound_speed_mps  = UCNL_WPHX_FWTR_SOUND_SPEED_MPS;

// GNSS lat, lon, speed and course
float gnss_lat_rad = INVALID_FLOAT;
float gnss_lon_rad = INVALID_FLOAT;
float gnss_spd_mps = INVALID_FLOAT;
float gnss_crs_deg = INVALID_FLOAT;

float gnss_lat_deg = INVALID_FLOAT;
float gnss_lon_deg = INVALID_FLOAT;

// Base modem's depth, water temperature and supply voltage
float own_dpt_m    = INVALID_FLOAT;
float own_tmp_deg  = INVALID_FLOAT;
float own_bat_v    = INVALID_FLOAT;

// Local coordinate system origin, lat and lon
float cc_lat_rad = INVALID_FLOAT;
float cc_lon_rad = INVALID_FLOAT;

// Previous measurement point lat and lon
float msm_lat_rad = INVALID_FLOAT;
float msm_lon_rad = INVALID_FLOAT;

// Minimal base size for the top and bottom modems relative placement
float min_base_size_m = INVALID_FLOAT;

// Measured/received remote data
float rem_ptime_s = INVALID_FLOAT;
float rem_dpt_m   = INVALID_FLOAT;
float rem_tmp_deg = INVALID_FLOAT;
float rem_bat_v   = INVALID_FLOAT;

// Resulting remote's location and quality (DRMS)
float rem_lat_rad = INVALID_FLOAT;
float rem_lon_rad = INVALID_FLOAT;
float rem_drms_m  = INVALID_FLOAT;
float rem_lat_deg = INVALID_FLOAT;
float rem_lon_deg = INVALID_FLOAT;

float rem_azm_deg    = INVALID_FLOAT;
float s_range_m      = INVALID_FLOAT;
float s_range_proj_m = INVALID_FLOAT;
float r_range_m      = INVALID_FLOAT;
float d_dpt_m        = INVALID_FLOAT;
float move_m         = INVALID_FLOAT;

float x_m, y_m, dst;

// System state machine's variables
bool own_amb_data_updated   = false;
bool own_location_updated   = false;
bool rem_request_engaged    = false;
bool rem_request_in_process = false;
bool rem_data_updated       = false;
bool rem_request_enabled    = false;
bool uwave_setup_done       = false;
bool uwave_setup_queried    = false;
bool loc_tmo                = false;
bool rem_tmo                = false;
int gnss_data_valid         = 0;
long rem_req_ts, ts;
bool needsRedraw            = false;


#ifdef USE_LCD

void drawScreen() {
  lcd.clear();

  if (screenID == 0)
  {
    lcd.print("DST  ");
    if ((gnss_data_valid) && !IS_F_IV(r_range_m))
    {
      lcd.print((int)(r_range_m <= 999 ? r_range_m : 999));
      lcd.print("m");
    }
    else
      lcd.print("---");

    lcd.setCursor(10, 0);
    lcd.print("|AZM ");
    if ((gnss_data_valid) && !IS_F_IV(rem_azm_deg))
    {
      lcd.print((int)rem_azm_deg);
      lcd.print("\xDF");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 1);
    lcd.print("DRMS ");
    if (!IS_F_IV(rem_drms_m) && (rem_drms_m <= DRMS_THRESHOLD_M))
    {
      lcd.print(rem_drms_m, 1);
      lcd.print("m");
    }
    else
      lcd.print("---");

    lcd.setCursor(10, 1);
    lcd.print("|CRS ");
    if (gnss_data_valid)
    {
      lcd.print((int)gnss_crs_deg);
      lcd.print("\xDF");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 2);
    lcd.print("GNSS ");
    if (gnss_data_valid)
      lcd.print("OK");
    else
      lcd.print("WAIT");

    lcd.setCursor(10, 2);
    lcd.print("|DPT ");
    if (!IS_F_IV(rem_dpt_m))
    {
      lcd.print((int)rem_dpt_m);
      lcd.print("m");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 3);
    lcd.print("MOVE ");
    if (!IS_F_IV(move_m))
    {
      lcd.print((int)move_m);
      lcd.print("m");
    }
    else
      lcd.print("---");

    lcd.setCursor(10, 3);
    lcd.print("|TMO --");
    lcd.setCursor(15, 3);
    if (loc_tmo)
      lcd.print("L");
    if (rem_tmo)
      lcd.print("R");
  }
  else if (screenID == 1)
  {
    lcd.print("GLAT: ");
    if (gnss_data_valid)
    {
      lcd.print(gnss_lat_deg, 6);
      lcd.print("\xDF");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 1);
    lcd.print("GLON: ");
    if (gnss_data_valid)
    {
      lcd.print(gnss_lon_deg, 6);
      lcd.print("\xDF");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 2);
    lcd.print("RLAT: ");
    if (!IS_F_IV(rem_lat_deg))
    {
      lcd.print(rem_lat_deg, 6);
      lcd.print("\xDF");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 3);
    lcd.print("RLON: ");
    if (!IS_F_IV(rem_lon_deg))
    {
      lcd.print(rem_lon_deg, 6);
      lcd.print("\xDF");
    }
    else
      lcd.print("---");
  }
  else if (screenID == 2)
  {
    lcd.print("SOS    : ");
    lcd.print(sound_speed_mps, 1);
    lcd.print(" m/s");

    lcd.setCursor(0, 1);
    lcd.print("WTMP(R): ");
    if (!IS_F_IV(rem_tmp_deg))
    {
      lcd.print(rem_tmp_deg, 1);
      lcd.print(" \xDF");
      lcd.print("C");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 2);
    lcd.print("WTMP(S): ");
    if (!IS_F_IV(own_tmp_deg))
    {
      lcd.print(own_tmp_deg, 1);
      lcd.print(" \xDF");
      lcd.print("C");
    }
    else
      lcd.print("---");

    lcd.setCursor(0, 3);
    lcd.print("BAT R/S: ");
    if (!IS_F_IV(rem_bat_v))
    {
      lcd.print(rem_bat_v, 1);
    }
    else
      lcd.print("----");

    lcd.print("/");
    if (!IS_F_IV(own_bat_v))
    {
      lcd.print(own_bat_v, 1);
    }
    else
      lcd.print("----");

    lcd.print(" V");
  }
}

#endif

void setup ()
{
  delay(100);
  pinMode(UWAVE_CMD_PIN, OUTPUT);
  digitalWrite(UWAVE_CMD_PIN, HIGH);
  delay(200);

#ifdef USE_SERIAL_OUT
  Serial.begin(9600);
#endif

  Serial1.begin(9600);
  Serial2.begin(9600);

  UCNL_NMEA_InitStruct(&gnssParser, gnss_in_buffer, UART_IN_BUFFER_SIZE, gnssSntIDs, GNSS_SNT_IDS_SIZE);
  UCNL_NMEA_InitStruct(&uwaveParser, uwave_in_buffer, UART_IN_BUFFER_SIZE, uwaveSntIDs, uWAVE_SNT_IDS_SIZE);
  UCNL_VLBL_ResetStructs(&pointsRing, &heapsRing);

  rcRequestData.txChID = REMOTE_TX_ID;
  rcRequestData.rxChID = REMOTE_RX_ID;
  rcRequestData.rcCmdID = RC_DPT_GET;

#ifdef USE_SERIAL_OUT
  Serial.print(F("Starting "));
  Serial.print(FW_MONIKER);
  Serial.print(F(", v"));
  Serial.print(FW_VERSION_MAJOR);
  Serial.print(F("."));
  Serial.println(FW_VERSION_MINOR);
#endif

#ifdef USE_LCD
  lcd.begin(20, 4);
  lcd.setCursor(0, 0);
  lcd.print(FW_MONIKER);
  lcd.setCursor(0, 1);
  lcd.print(F("v"));
  lcd.print(FW_VERSION_MAJOR);
  lcd.print(F("."));
  lcd.print(FW_VERSION_MINOR);
  lcd.setCursor(0, 3);
  lcd.print(F("       HELLO!       "));
  delay(1000);
  lcd.clear();

  drawScreen();

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, LOW);

#endif
}

void loop ()
{
  if (Serial1.available()) {
    byte b = Serial1.read();
    parserResult = UCNL_NMEA_Process_Byte(&gnssParser, b);

    if (parserResult == UCNL_NMEA_RESULT_PACKET_READY) {
      if (gnssParser.sntID == UCNL_NMEA_RMC_SNT_ID) {
        if (UCNL_NMEA_Parse_RMC(&gnssRMCData, gnssParser.buffer, gnssParser.idx)) {

          gnss_lat_deg = gnssRMCData.latitude_deg;
          gnss_lon_deg = gnssRMCData.longitude_deg;
          gnss_lat_rad = UCNL_NAV_DEG2RAD(gnss_lat_deg);
          gnss_lon_rad = UCNL_NAV_DEG2RAD(gnss_lon_deg);
          gnss_spd_mps = gnssRMCData.speed_kmh / 3.6;
          gnss_crs_deg = gnssRMCData.course_deg;
          own_location_updated = true;
          gnss_data_valid = GNSS_DATA_VALIDITY_HYST;

#ifdef USE_SERIAL_OUT
          Serial.print("Own loc: ");
          Serial.print(gnss_lat_deg, 6);
          Serial.print(", ");
          Serial.println(gnss_lon_deg, 6);
#endif
        }
        else {
          if (gnss_data_valid > 0)
          {
            gnss_data_valid--;
#ifdef USE_SERIAL_OUT
            Serial.print("GNSS age: ");
            Serial.print(GNSS_DATA_VALIDITY_HYST - gnss_data_valid);
            Serial.println(" sec");
#endif
          }
          else
          {
#ifdef USE_SERIAL_OUT
            Serial.println("GNSS waiting..");
#endif
          }
        }
        needsRedraw = true;
      }
      UCNL_NMEA_Release(&gnssParser);
    }
  }

  if (Serial2.available()) {
    byte b = Serial2.read();
    parserResult = UCNL_NMEA_Process_Byte(&uwaveParser, b);
    if (parserResult == UCNL_NMEA_RESULT_PACKET_READY) {
      loc_tmo = false;

      if ((uwaveParser.sntID == uWAVE_NMEA_UWV0_SNT_ID) &&
          (uWAVE_Parse_ACK(&ackData, uwaveParser.buffer, uwaveParser.idx))) {
        if (ackData.sentenceID == IC_H2D_RC_REQUEST) {
          if (ackData.errCode == LOC_ERR_NO_ERROR)
            rem_request_in_process = true;

          rem_request_engaged = false;
        }
        else if (ackData.sentenceID == IC_H2D_AMB_DTA_CFG) {
          if (ackData.errCode == LOC_ERR_NO_ERROR)
            uwave_setup_done = true;
          else
            uwave_setup_queried = false;
        }
      }
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWV3_SNT_ID) &&
               (uWAVE_Parse_RC_RESPONSE(&rcResponseData, uwaveParser.buffer, uwaveParser.idx))) {
        if (rcResponseData.isPropTime)
          rem_ptime_s = abs(rcResponseData.propTime_sec);

        if (rcResponseData.isValue) {
          if (rcResponseData.rcCmdID == RC_DPT_GET)
            rem_dpt_m = rcResponseData.value;
          else if (rcResponseData.rcCmdID == RC_TMP_GET)
            rem_tmp_deg = rcResponseData.value;
          else if (rcResponseData.rcCmdID == RC_BAT_V_GET)
            rem_bat_v = rcResponseData.value;
        }
        rem_data_updated = true;
        rem_tmo = false;
      }
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWV4_SNT_ID) &&
               (uWAVE_Parse_RC_TIMEOUT(&rcTimeoutData, uwaveParser.buffer, uwaveParser.idx))) {
        rem_request_in_process = false;
        rem_tmo = true;
      }
      else if ((uwaveParser.sntID == uWAVE_NMEA_UWV7_SNT_ID) &&
               (uWAVE_Parse_AMB_DTA(&ambData, uwaveParser.buffer, uwaveParser.idx))) {
        if (ambData.isDpt && ambData.isTemp && ambData.isBat) {
          own_dpt_m = ambData.dpt_m;
          own_tmp_deg = ambData.temp_C;
          own_bat_v = ambData.batVoltage_V;
          own_amb_data_updated = true;
        }
      }
      UCNL_NMEA_Release(&uwaveParser);
    }
  }

  if (!uwave_setup_done && !uwave_setup_queried) {
    uwave_out_buffer_idx = 0;
    UCNL_STR_WriteStr(uwave_out_buffer, &uwave_out_buffer_idx, uWAVE_AMB_DTA_CFG_SNT);
    Serial2.write(uwave_out_buffer, uwave_out_buffer_idx);
    uwave_setup_queried = true;
  }

  if (own_location_updated) {
    if (own_amb_data_updated) {
      if (rem_data_updated) {
        if (!IS_F_IV(rem_dpt_m) &&
            !IS_F_IV(rem_tmp_deg) &&
            !IS_F_IV(own_tmp_deg)) {
          sound_speed_mps = UCNL_WPHX_speed_of_sound_UNESCO_calc((rem_tmp_deg + own_tmp_deg) / 2.0,
                            UCNL_WPHX_ATM_PRESSURE_MBAR,
                            WATER_SALINITY_PSU);

#ifdef USE_SERIAL_OUT
          Serial.print("SOS update: ");
          Serial.println(sound_speed_mps, 1);
#endif
        }
        s_range_m = rem_ptime_s * sound_speed_mps;

        if (!IS_F_IV(rem_dpt_m) && !IS_F_IV(own_dpt_m)) {
          d_dpt_m = abs(rem_dpt_m - own_dpt_m);
          s_range_proj_m = d_dpt_m > s_range_m ? sqrt(s_range_m * s_range_m - d_dpt_m * d_dpt_m) : s_range_m;

          if (IS_F_IV(min_base_size_m))
            min_base_size_m = s_range_proj_m > MIN_BASE_SIZE_M ? s_range_proj_m * BASE_SIZE_FACTOR : MIN_BASE_SIZE_M;

          UCNL_NAV_GetDeltasByGeopoints_WGS84(cc_lat_rad, cc_lon_rad,
                                              gnss_lat_rad, gnss_lon_rad,
                                              &y_m, &x_m);

#ifdef USE_SERIAL_OUT
          Serial.print("MSM: ");
          Serial.print(x_m);
          Serial.print(", ");
          Serial.print(y_m);
          Serial.print(", ");
          Serial.println(s_range_proj_m);
#endif

          UCNL_VLBL_Clusterize(&pointsRing, &heapsRing, x_m, y_m, s_range_proj_m);

          if (heapsRing.cnt >= 2) {
            if (heapsRing.drms_best <= DRMS_THRESHOLD_M) {
              UCNL_NAV_PointOffset_WGS84(cc_lat_rad, cc_lon_rad, heapsRing.y_best, heapsRing.x_best, &rem_lat_rad,  &rem_lon_rad);
              rem_drms_m = heapsRing.drms_best;
              rem_lat_deg = UCNL_NAV_RAD2DEG(rem_lat_rad);
              rem_lon_deg = UCNL_NAV_RAD2DEG(rem_lon_rad);

#ifdef USE_SERIAL_OUT
              Serial.print("Remote: ");
              Serial.print(rem_lat_deg, 6);
              Serial.print(",");
              Serial.print(rem_lon_deg, 6);
              Serial.print(", ");
              Serial.print(rem_dpt_m, 1);
              Serial.print(", ");
              Serial.println(rem_drms_m, 1);
#endif
            }
          }
        }

        msm_lat_rad = gnss_lat_rad;
        msm_lon_rad = gnss_lon_rad;
        rem_data_updated = false;
        rem_request_in_process = false;
      }
      own_amb_data_updated = false;

      needsRedraw = true;
    }

    if (!rem_request_in_process && uwave_setup_done) {
      if (!rem_request_engaged) {
        rem_request_enabled = false;
        if (IS_F_IV(msm_lat_rad) &&
            IS_F_IV(msm_lon_rad)) {
          cc_lat_rad = gnss_lat_rad;
          cc_lon_rad = gnss_lon_rad;
          rem_request_enabled = true;

#ifdef USE_SERIAL_OUT
          Serial.print("CSO: ");
          Serial.print(cc_lat_rad, 6);
          Serial.print(", ");
          Serial.println(cc_lon_rad, 6);
#endif
        }
        else {
          dst = UCNL_NAV_HaversineInverse(msm_lat_rad, msm_lon_rad, gnss_lat_rad, gnss_lon_rad);
          if (dst >= min_base_size_m) {
            rem_request_enabled = true;
            move_m = INVALID_FLOAT;
          }
          else {
            move_m = min_base_size_m - dst;

#ifdef USE_SERIAL_OUT
            Serial.print("Move: ");
            Serial.print(move_m);
            Serial.println(" m");
#endif
          }
        }

        if (rem_request_enabled) {
          if (IS_F_IV(rem_dpt_m))
            rcRequestData.rcCmdID = RC_DPT_GET;
          else if (IS_F_IV(rem_tmp_deg))
            rcRequestData.rcCmdID = RC_TMP_GET;
          else if (IS_F_IV(rem_bat_v))
            rcRequestData.rcCmdID = RC_BAT_V_GET;
          else {
            rcRequestData.rcCmdID = rcRequestData.rcCmdID == RC_DPT_GET ? RC_TMP_GET : (rcRequestData.rcCmdID == RC_TMP_GET ? RC_BAT_V_GET : RC_DPT_GET);
          }

          uWAVE_Build_RC_REQUEST(&rcRequestData, uwave_out_buffer, UART_OUT_BUFFER_SIZE, &uwave_out_buffer_idx);
          Serial2.write(uwave_out_buffer, uwave_out_buffer_idx);
          rem_request_engaged = true;
          rem_req_ts = millis();
        }
      }
    }
    else {
      ts = millis();
      if (ts - rem_req_ts > LOC_TIMEOUT_MS) {
        rem_request_in_process = false;
        rem_request_engaged = false;
        loc_tmo = true;
      }
    }

    if (!IS_F_IV(rem_lat_rad) &&
        !IS_F_IV(rem_lon_rad)) {
      rem_azm_deg = UCNL_NAV_RAD2DEG(UCNL_NAV_HaversineInitialBearing(gnss_lat_rad, gnss_lon_rad, rem_lat_rad, rem_lon_rad));
      r_range_m = UCNL_NAV_HaversineInverse(gnss_lat_rad, gnss_lon_rad, rem_lat_rad, rem_lon_rad);

#ifdef USE_SERIAL_OUT
      Serial.print("2Remote: ");
      Serial.print(r_range_m, 0);
      Serial.print(" m,");
      Serial.print(rem_azm_deg, 0);
      Serial.println(" \xDF");
#endif
    }

    own_location_updated = false;
  }

#ifdef USE_LCD

  ts = millis();
  if (ts - btn_check_ts >= BTN_DEBOUNCE_MS) {
    btn_check_ts = ts;
    if (digitalRead(BUTTON_PIN) == BTN_ACTIVE) {
      screenID = (screenID + 1) % SCREEN_NUM;
      needsRedraw = true;
    }
  }

  if (needsRedraw) {
    drawScreen();
    needsRedraw = false;
  }

#endif

}
