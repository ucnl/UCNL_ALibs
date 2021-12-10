/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com
*/

#ifndef _UCNL_NMEA_
#define _UCNL_NMEA_

#define UCNL_NMEA_SNT_STR              '$'
#define UCNL_NMEA_SNT_END              '\n'
#define UCNL_NMEA_SNT_END1             '\r'
#define UCNL_NMEA_PAR_SEP              ','
#define UCNL_NMEA_CHK_SEP              '*'

#define UCNL_NMEA_DATA_NOT_VALID       'V'
#define UCNL_NMEA_SOUTH_SIGN           'S'
#define UCNL_NMEA_WEST_SIGN            'W'
#define UCNL_NMEA_TD_VALID             'A'
#define UCNL_NMEA_PSENTENCE_SYMBOL     'P'
#define UCNL_NMEA_PMODE_DATA_NOT_VALID 'N'

#define UCNL_NMEA_MIN_LEN              (8)

#define NMEA_GSA_PRNS_NUM              (12)
#define NMEA_GSV_SATS_NUM              (4)

typedef struct {
  byte* buffer;
  byte  buffer_size;
  byte  idx;
  bool  isReady;
  bool  isStarted;
  bool  isPSentence;
  byte  chk_act;
  byte  chk_dcl;
  byte  chk_dcl_idx;
  bool  chk_present;
  int   tkrID;
  long  sntID;
  long* sntIDs;
  byte  sntIDs_size;

} UCNL_NMEA_State_Struct;

typedef enum {
  UCNL_NMEA_RESULT_PACKET_READY          = 0,
  UCNL_NMEA_RESULT_BYPASS_BYTE           = 1,
  UCNL_NMEA_RESULT_PACKET_STARTED        = 2,
  UCNL_NMEA_RESULT_PACKET_PROCESS        = 3,
  UCNL_NMEA_RESULT_PACKET_CHECKSUM_ERROR = 4,
  UCNL_NMEA_RESULT_PACKET_TOO_BIG        = 5,
  UCNL_NMEA_RESULT_PACKET_SKIPPING       = 6,
  UCNL_NMEA_RESULT_UNKNOWN
} UCNL_NMEA_Result_Enum;

typedef enum {
  NMEA_GGA_QTY_NO_FIX              = 0,
  NMEA_GGA_QTY_SINGLE_POINT        = 1,
  NMEA_GGA_QTY_PRANGE_DIFFERENTIAL = 2,
  NMEA_GGA_QTY_RTK_FIXED           = 4,
  NMEA_GGA_QTY_RTK_FLOATING        = 5,
  NMEA_GGA_QTY_DEAD_RECKONING      = 6,
  NMEA_GGA_QTY_MANUAL              = 7,
  NMEA_GGA_QTY_SIMULATOR           = 8,
  NMEA_GGA_QTY_WAAS_SBAS           = 9,
  NMEA_GGA_QTY_UNKNOWN
} NMEA_GGA_QTY_IND_Enum;

typedef enum {
  NMEA_PSMODE_AUTONOMOUS     = 'A',
  NMEA_PSMODE_DIFFERENTIAL   = 'D',
  NMEA_PSMODE_DEAD_RECKONING = 'E',
  NMEA_PSMODE_MANUAL         = 'M',
  NMEA_PSMODE_INVALID        = 'N',
  NMEA_PSMODE_UNKNOWN  
} NMEA_PSMODE_IND_Enum;

// Standard known sentences result structures
typedef struct {
  bool isValid;
  byte hour;
  byte minute;
  float second;
  byte date;
  byte month;
  byte year;
  float latitude_deg;
  float longitude_deg;
  float speed_kmh;
  float course_deg;
} UCNL_NMEA_RMC_RESULT_Struct;

typedef struct {
  bool isValid;
  byte hour;
  byte minute;
  float second;
  float latitude_deg;
  float longitude_deg;
  float hdop;
  float gsep;
  float dgps_rec_age;
  byte sats_in_use;
  byte gnss_qly_ind;
  float orth_height;
  byte datumID;
} UCNL_NMEA_GGA_RESULT_Struct;

typedef struct {
  bool isValid;
  byte hour;
  byte minute;
  float second;
  float latitude_deg;
  float longitude_deg;
} UCNL_NMEA_GLL_RESULT_Struct;

typedef struct {
  bool isValid;
  byte mode;
  byte fix_type;
  byte prns[NMEA_GSA_PRNS_NUM];
  float pdop;
  float hdop;
  float vdop;
  byte systemID;
} UCNL_NMEA_GSA_RESULT_Struct;

typedef struct {
  bool isValid;
  byte total_messages;
  byte current_message;
  byte sats_in_view;
  
  int PRNNumbers[NMEA_GSV_SATS_NUM];
  int Elevations[NMEA_GSV_SATS_NUM];
  int Azimuths[NMEA_GSV_SATS_NUM];
  int SNRs[NMEA_GSV_SATS_NUM];

  byte satDataNum;  
  
} UCNL_NMEA_GSV_RESULT_Struct;

typedef struct {
  bool isValid;
  float track_true_deg;
  bool is_magnetic;
  float track_magnetic_deg;
  float speed_knots;
  float speed_kmh;
} UCNL_NMEA_VTG_RESULT_Struct;

typedef struct {
  bool isValid;
  float track_true_deg;
} UCNL_NMEA_HDT_RESULT_Struct;

typedef struct {
  bool isValid;
  float magnetic_heading_deg;
  bool is_magnetic_variation;
  float magnetic_variation;
} UCNL_NMEA_HDG_RESULT_Struct;

typedef struct {
  bool isValid;
  byte hour;
  byte minute;
  float second;
  byte day; // 01..31
  byte month;
  int year; // 4-digit
  int t_zone_offset_hours; // +/- 13 hours
  byte t_zone_offset_minutes; // 0..59
} UCNL_NMEA_ZDA_RESULT_Struct;

typedef struct {
  bool isValid;
  float mean_water_temperature_c;
} UCNL_NMEA_MTW_RESULT_Struct;


#define UCNL_NMEA_RMC_SNT_ID (0x524D43)
#define UCNL_NMEA_GGA_SNT_ID (0x474741)
#define UCNL_NMEA_GLL_SNT_ID (0x474C4C)
#define UCNL_NMEA_GSA_SNT_ID (0x475341)
#define UCNL_NMEA_GSV_SNT_ID (0x475356)
#define UCNL_NMEA_VTG_SNT_ID (0x565447)
#define UCNL_NMEA_HDT_SNT_ID (0x484454)
#define UCNL_NMEA_HDG_SNT_ID (0x484447)
#define UCNL_NMEA_ZDA_SNT_ID (0x5A4441)
#define UCNL_NMEA_MTW_SNT_ID (0x4D5457)

#define UCNL_NMEA_IS_VALID_DATE(b)    (((b) >= 1) && ((b) <= 31))
#define UCNL_NMEA_IS_VALID_MONTH(b)   (((b) >= 1) && ((b) <= 12))
#define UCNL_NMEA_IS_VALID_YEAR(b)    (((b) >= 0) && ((b) <= 99))
#define UCNL_NMEA_IS_VALID_HOUR(b)    (((b) >= 0) && ((b) <= 23))
#define UCNL_NMEA_IS_VALID_MINSEC(b)  (((b) >= 0) && ((b) < 60))
#define UCNL_NMEA_IS_VALID_LATDEG(l)  (((l) >= -90) && ((l) <= 90))
#define UCNL_NMEA_IS_VALID_LONDEG(l)  (((l) >= -180) && ((l) <= 180))


void UCNL_NMEA_InitStruct(UCNL_NMEA_State_Struct* uState, byte* buffer, byte buffer_size, long* sntIDs, byte sntIDs_size);
void UCNL_NMEA_Release(UCNL_NMEA_State_Struct* uState);

UCNL_NMEA_Result_Enum UCNL_NMEA_Process_Byte(UCNL_NMEA_State_Struct* uState, byte newByte);
bool                  UCNL_NMEA_Get_NextParam(const byte* buffer, byte fromIdx, byte size, byte* stIdx, byte* ndIdx);
void                  UCNL_NMEA_CheckSum_Update(byte* buffer, byte size);

// Standard parsers
bool                  UCNL_NMEA_Parse_RMC(UCNL_NMEA_RMC_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_GGA(UCNL_NMEA_GGA_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_GLL(UCNL_NMEA_GLL_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_VTG(UCNL_NMEA_VTG_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_HDT(UCNL_NMEA_HDT_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_HDG(UCNL_NMEA_HDG_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_MTW(UCNL_NMEA_MTW_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_GSA(UCNL_NMEA_GSA_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_GSV(UCNL_NMEA_GSV_RESULT_Struct* rdata, const byte* buffer, byte idx);
bool                  UCNL_NMEA_Parse_ZDA(UCNL_NMEA_ZDA_RESULT_Struct* rdata, const byte* buffer, byte idx);

#endif
