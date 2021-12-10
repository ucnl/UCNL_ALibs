/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#ifndef _UCNL_UWAVE_
#define _UCNL_UWAVE_

#define PUWV_PREFIX                "$PUWV\0"

#define IC_D2H_ACK                 '0'        // $PUWV0,cmdID,errCode
#define IC_H2D_SETTINGS_WRITE      '1'        // $PUWV1,rxChID,txChID,styPSU,isCmdMode,isACKOnTXFinished,gravityAcc

// RC mode
#define IC_H2D_RC_REQUEST          '2'        // $PUWV2,txChID,rxChID,rcCmdID
#define IC_D2H_RC_RESPONSE         '3'        // $PUWV3,txChID,rcCmdID,[propTime_seс],snr,[value],[azimuth]
#define IC_D2H_RC_TIMEOUT          '4'        // $PUWV4,txChID,rcCmdID
#define IC_D2H_RC_ASYNC_IN         '5'        // $PUWV5,rcCmdID,snr,[azimuth]
//

// AMB data
#define IC_H2D_AMB_DTA_CFG         '6'        // $PUWV6,isSaveInFlash,periodMs,isPrs,isTemp,isDpt,isBatV
#define IC_D2H_AMB_DTA             '7'        // $PUWV7,prs_mBar,temp_C,dpt_m,batVoltage_V
//

// For interfacing with uWave USBL devices
#define IC_H2D_INC_DTA_CFG         '8'        // $PUWV8,isSaveInFlash,periodMs,
#define IC_D2H_INC_DTA             '9'        // $PUWV9,[reserved empty field],pitch,roll

// packet mode
#define IC_H2D_PT_SETTINGS_READ    'D'        // $PUWVD,reserved
#define IC_D2H_PT_SETTINGS         'E'        // $PUWVE,isPTMode,ptAddress
#define IC_H2H_PT_SETTINGS_WRITE   'F'        // $PUWVF,isSaveInFlash,isPTMode,ptAddress

#define IC_H2D_PT_SEND             'G'        // $PUWVG,tareget_ptAddress,[maxTries],dataPacket
#define IC_D2H_PT_FAILED           'H'        // $PUWVH,tareget_ptAddress,triesTaken,dataPacket
#define IC_D2H_PT_DLVRD            'I'        // $PUWVI,tareget_ptAddress,triesTaken,dataPacket
#define IC_D2H_PT_RCVD             'J'        // $PUWVJ,sender_ptAddress,dataPacket

#define IC_H2D_PT_ITG              'K'        // $PUWVK,target_ptAddress,pt_itg_dataID
#define IC_D2H_PT_TMO              'L'        // $PUWVL,target_ptAddress,pt_itg_dataID
#define IC_D2H_PT_ITG_RESP         'M'        // $PUWVM,target_ptAddress,pt_itg_dataID,[dataValue],pTime,[azimuth]
//

// AQPNG Mode
#define IC_H2D_AQPNG_SETTINGS_READ 'N'        // $PUWVN,reserved
#define IC_HDH_AQPNG_SETTINGS      'O'        // $PUWVO,[isSaveInFlash],AQPN_ModeID,[periodMs],[rcCmdID],[rcTxID],[rcRxID],[isPT],[pt_targetAddr]
//

#define IC_H2D_DINFO_GET           '?'         // $PUWV?,reserved
#define IC_D2H_DINFO               '!'         // $PUWV!,serialNumber,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,isCmdMode

#define IC_D2H_UNKNOWN             '-'


#define uWAVE_NMEA_UWV0_SNT_ID     (0x55575630)
#define uWAVE_NMEA_UWV3_SNT_ID     (0x55575633)
#define uWAVE_NMEA_UWV4_SNT_ID     (0x55575634)
#define uWAVE_NMEA_UWV5_SNT_ID     (0x55575635)
#define uWAVE_NMEA_UWV7_SNT_ID     (0x55575637)
#define uWAVE_NMEA_UWV9_SNT_ID     (0x55575639)
#define uWAVE_NMEA_UWVE_SNT_ID     (0x55575645)
#define uWAVE_NMEA_UWVH_SNT_ID     (0x55575648)
#define uWAVE_NMEA_UWVI_SNT_ID     (0x55575649)
#define uWAVE_NMEA_UWVJ_SNT_ID     (0x5557564A)
#define uWAVE_NMEA_UWVL_SNT_ID     (0x5557564C)
#define uWAVE_NMEA_UWVM_SNT_ID     (0x5557564D)
#define uWAVE_NMEA_UWVI_SNT_ID     (0x5557564F)
#define uWAVE_NMEA_UWV_EXCL_SNT_ID (0x55575621)

#define uWAVE_PKT_MAX_SIZE         (64)


typedef enum uWAVE_AQPNG_Mode_Enum
{
  AQPNG_DISABLED = 0,
  AQPNG_PINGER   = 1,
  AQPNG_MASTER   = 2,
  AQPNG_INVALID
};

typedef enum uWAVE_RC_CODES_Enum
{
  RC_PING        = 0,
  RC_PONG        = 1,
  RC_DPT_GET     = 2,
  RC_TMP_GET     = 3,
  RC_BAT_V_GET   = 4,
  RC_ERR_NSUP    = 5,
  RC_ACK         = 6,
  RC_USR_CMD_000 = 7,
  RC_USR_CMD_001 = 8,
  RC_USR_CMD_002 = 9,
  RC_USR_CMD_003 = 10,
  RC_USR_CMD_004 = 11,
  RC_USR_CMD_005 = 12,
  RC_USR_CMD_006 = 13,
  RC_USR_CMD_007 = 14,
  RC_USR_CMD_008 = 15,
  RC_MSG_ASYNC   = 16,
  RC_INVALID
};

typedef enum uWAVE_DataID_Enum
{
  DID_DPT = 0,
  DID_TMP = 1,
  DID_BAT = 2,
  DID_INVALID
};

typedef enum uWAVE_ERR_CODES_Enum
{
  LOC_ERR_NO_ERROR              = 0,
  LOC_ERR_INVALID_SYNTAX        = 1,
  LOC_ERR_UNSUPPORTED           = 2,
  LOC_ERR_TRANSMITTER_BUSY      = 3,
  LOC_ERR_ARGUMENT_OUT_OF_RANGE = 4,
  LOC_ERR_INVALID_OPERATION     = 5,
  LOC_ERR_UNKNOWN_FIELD_ID      = 6,
  LOC_ERR_VALUE_UNAVAILABLE     = 7,
  LOC_ERR_RECEIVER_BUSY         = 8,
  LOC_ERR_TX_BUFFER_OVERRUN     = 9,
  LOC_ERR_CHKSUM_ERROR          = 10,
  LOC_ERR_TX_FINISHED           = 11,
  LOC_ACK_BEFORE_STANDBY        = 12,
  LOC_ACK_AFTER_WAKEUP          = 13,
  LOC_ERR_SVOLTAGE_TOO_HIGH     = 14,
  LOC_ERR_UNKNOWN
};

// Sentence descriptors
typedef struct
{
  byte rxChID;
  byte txChID;
  float styPSU;
  bool isCmdMode;
  bool isACKOnTXFinished;
  float gravityAcc;
} uWAVE_SETTINGS_WRITE_Struct;

typedef struct
{
  byte sentenceID;
  uWAVE_ERR_CODES_Enum errCode;
} uWAVE_ACK_RESULT_Struct;

typedef struct
{
  byte txChID;
  byte rxChID;
  uWAVE_RC_CODES_Enum rcCmdID;
} uWAVE_RC_REQUEST_Struct;

typedef struct
{
  byte txChID;
  uWAVE_RC_CODES_Enum rcCmdID;
  bool isPropTime;
  float propTime_sec;
  float MSR_dB;
  bool isValue;
  float value;
  bool isAzimuth;
  float azimuth;
} uWAVE_RC_RESPONSE_Struct;

typedef struct
{
  byte txChID;
  uWAVE_RC_CODES_Enum rcCmdID;
} uWAVE_RC_TIMEOUT_Struct;

typedef struct
{
  uWAVE_RC_CODES_Enum rcCmdID;
  float MSR_dB;
  bool isAzimuth;
  float azimuth;
} uWAVE_RC_ASYNC_IN_Struct;

typedef struct
{
  bool isSaveInFlash;
  int periodMs;
  bool isPrs;
  bool isTemp;
  bool isDpt;
  bool isBatV;
} uWAVE_AMB_DTA_CFG_Struct;

typedef struct
{
  bool isPrs;
  float prs_mBar;
  bool isTemp;
  float temp_C;
  bool isDpt;
  float dpt_m;
  bool isBat;
  float batVoltage_V;
} uWAVE_AMB_DTA_Struct;

typedef struct
{
  bool isSaveInFlash;
  int periodMs;
} uWAVE_INC_DTA_CFG_Struct;

typedef struct
{
  bool isHeading;
  float heading;
  bool isPitch;
  float pitch;
  bool isRoll;
  float roll;
} uWAVE_INC_DTA_Struct;

typedef struct
{
  bool isSaveInFlash;
  bool isPtEnabled;
  byte ptAddress;
} uWAVE_PT_SETTINGS_Struct;

typedef struct
{
  byte ptAddress;
  byte tries;
  bool isAzimuth;
  float azimuth;
  byte* dataPacket;
  byte dataPacketSize;
} uWAVE_PT_PACKET_Struct;

typedef struct
{
  byte ptAddress;
  uWAVE_DataID_Enum pt_itg_dataID;
} uWAVE_PT_ITG_Struct;

typedef struct
{
  byte target_ptAddress;
  uWAVE_DataID_Enum pt_itg_dataID;
  bool isValue;
  float dataValue;
  bool isPTime;
  float pTime;
  bool isAzimuth;
  float azimuth;
} uWAVE_PT_ITG_RESP_Struct;

typedef struct
{
  bool isSaveInFlash;
  uWAVE_AQPNG_Mode_Enum aqpng_Mode;
  bool isPeriod;
  int periodMs;
  bool isDataID;
  uWAVE_DataID_Enum dataID;
  byte rcTxID;
  byte rcRxID;
  bool isPT;
  byte pt_targetAddr;
} uWAVE_AQPNG_SETTINGS_Struct;

typedef struct
{
  byte* serialNumber;
  byte serialNumber_size;
  byte *sys_moniker;
  byte sys_moniker_size;
  int sys_version;
  byte* core_moniker;
  byte core_moniker_size;
  int core_version;
  float acBaudrate;
  byte rxChID;
  byte txChID;
  byte totalChannels;
  float styPSU;
  bool isPTSPresent;
  bool isCmdMode;
} uWAVE_DINFO_Struct;



// Parsers
bool uWAVE_Parse_ACK(uWAVE_ACK_RESULT_Struct* rdata, const byte* buffer, byte idx);

bool uWAVE_Parse_RC_RESPONSE(uWAVE_RC_RESPONSE_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_RC_TIMEOUT(uWAVE_RC_TIMEOUT_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_RC_ASYNC_IN(uWAVE_RC_ASYNC_IN_Struct* rdata, const byte* buffer, byte idx);

bool uWAVE_Parse_AMB_DTA(uWAVE_AMB_DTA_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_INC_DTA(uWAVE_INC_DTA_Struct* rdata, const byte* buffer, byte idx);

bool uWAVE_Parse_PT_SETTINGS(uWAVE_PT_SETTINGS_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_PT_FAILED(uWAVE_PT_PACKET_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_PT_DLVRD(uWAVE_PT_PACKET_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_PT_RCVD(uWAVE_PT_PACKET_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_PT_TMO(uWAVE_PT_ITG_Struct* rdata, const byte* buffer, byte idx);
bool uWAVE_Parse_PT_ITG_RESP(uWAVE_PT_ITG_RESP_Struct* rdata, const byte* buffer, byte idx);

bool uWAVE_Parse_AQPNG_SETTINGS(uWAVE_AQPNG_SETTINGS_Struct* rdata, const byte* buffer, byte idx);

bool uWAVE_Parse_DINFO(uWAVE_DINFO_Struct* rdata, const byte* buffer, byte idx);


// Sentence builders
void uWAVE_Build_SETTINGS_WRITE(uWAVE_SETTINGS_WRITE_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_RC_REQUEST(uWAVE_RC_REQUEST_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_AMB_DTA_CFG(uWAVE_AMB_DTA_CFG_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_INC_DTA_CFG(uWAVE_INC_DTA_CFG_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);

void uWAVE_Build_PT_SETTINGS_READ(byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_PT_SETTINGS_WRITE(uWAVE_PT_SETTINGS_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_PT_SEND(uWAVE_PT_PACKET_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_PT_ABORT_SEND(byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_PT_ITG(uWAVE_PT_ITG_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);

void uWAVE_Build_AQPNG_SETTINGS_READ(byte* buffer, byte bufferSize, byte* idx);
void uWAVE_Build_AQPNG_SETTINGS(uWAVE_AQPNG_SETTINGS_Struct* sdata, byte* buffer, byte bufferSize, byte* idx);

void uWAVE_Build_DINFO_GET(byte* buffer, byte bufferSize, byte* idx);

#endif
