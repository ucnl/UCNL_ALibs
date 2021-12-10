/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/


#include "Arduino.h"
#include "ucnl_str.h"
#include "ucnl_nmea.h"
#include "ucnl_uwave.h"


// Parsers
bool uWAVE_Parse_ACK(uWAVE_ACK_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWV0,sndID,errCode
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->sentenceID = char(buffer[stIdx]);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->errCode = (uWAVE_ERR_CODES_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_RC_RESPONSE(uWAVE_RC_RESPONSE_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWV3,txChID,rcCmdID,[propTime_seс],msr,[value],[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->txChID = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->rcCmdID = (uWAVE_RC_CODES_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          rdata->isPropTime = false;
        else
        {
          rdata->isPropTime = true;
          rdata->propTime_sec = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 4:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->MSR_dB = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 5:
        if (ndIdx < stIdx)
          rdata->isValue = true;
        else
        {
          rdata->isValue = true;
          rdata->value = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 6:
        if (ndIdx < stIdx)
          rdata->isAzimuth = true;
        else
        {
          rdata->isAzimuth = true;
          rdata->azimuth = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_RC_TIMEOUT(uWAVE_RC_TIMEOUT_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWV4,txChID,rcCmdID
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->txChID = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->rcCmdID = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_RC_ASYNC_IN(uWAVE_RC_ASYNC_IN_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWV5,rcCmdID,msr,[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->rcCmdID = (uWAVE_RC_CODES_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->MSR_dB = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          rdata->isAzimuth = false;
        else
        {
          rdata->isAzimuth = true;
          rdata->azimuth = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_AMB_DTA(uWAVE_AMB_DTA_Struct* rdata, const byte* buffer, byte idx)
{
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx > stIdx)
        {
          rdata->isPrs = true;
          rdata->prs_mBar = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isPrs = false;
        break;
      case 2:
        if (ndIdx > stIdx)
        {
          rdata->isTemp = true;
          rdata->temp_C = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isTemp = false;
        break;
      case 3:
        if (ndIdx > stIdx)
        {
          rdata->isDpt = true;
          rdata->dpt_m = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isDpt = false;
        break;
      case 4:
        if (ndIdx > stIdx)
        {
          rdata->isBat = true;
          rdata->batVoltage_V = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isBat = false;
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_INC_DTA(uWAVE_INC_DTA_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWV9,,pitch,roll
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx > stIdx)
        {
          rdata->isHeading = true;
          rdata->heading = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isHeading = false;
        break;
      case 2:
        if (ndIdx > stIdx)
        {
          rdata->isPitch = true;
          rdata->pitch = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isPitch = false;
        break;
      case 3:
        if (ndIdx > stIdx)
        {
          rdata->isRoll = true;
          rdata->roll = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->isRoll = false;
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_PT_SETTINGS(uWAVE_PT_SETTINGS_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVE,isPTMode,ptAddress
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->isPtEnabled = (bool)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->ptAddress = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_PT_FAILED(uWAVE_PT_PACKET_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVH,target_ptAddress,triesTaken,dataPacket
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->ptAddress = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->tries = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 3:
        if ((ndIdx < stIdx) ||
            (UCNL_STR_ReadHexStr(buffer, stIdx, ndIdx, rdata->dataPacket, uWAVE_PKT_MAX_SIZE, &(rdata->dataPacketSize)) != 0))
          result = false;
        break;
    }

    pIdx++;
  } while (result && !isNotLastParam);

  return result;
}

bool uWAVE_Parse_PT_DLVRD(uWAVE_PT_PACKET_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVI,tareget_ptAddress,[azimuth],triesTaken,dataPacket
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->ptAddress = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx > stIdx)
        {
          rdata->isAzimuth = true;
          rdata->azimuth = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->tries = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 4:
        if ((ndIdx < stIdx) || (UCNL_STR_ReadHexStr(buffer, stIdx, ndIdx, rdata->dataPacket, uWAVE_PKT_MAX_SIZE, &(rdata->dataPacketSize)) != 0))
          result = false;
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_PT_RCVD(uWAVE_PT_PACKET_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVJ,sender_ptAddress,[azimuth],dataPacket
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->ptAddress = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx > stIdx)
        {
          rdata->isAzimuth = true;
          rdata->azimuth = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 3:
        if ((ndIdx < stIdx) || (UCNL_STR_ReadHexStr(buffer, stIdx, ndIdx, rdata->dataPacket, uWAVE_PKT_MAX_SIZE, &(rdata->dataPacketSize)) != 0))
          result = false;
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_PT_TMO(uWAVE_PT_ITG_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVL,target_ptAddress,pt_itg_dataID
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->ptAddress = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->pt_itg_dataID = (uWAVE_DataID_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_PT_ITG_RESP(uWAVE_PT_ITG_RESP_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVM,target_ptAddress,pt_itg_dataID,[dataValue],pTime,[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->target_ptAddress = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->pt_itg_dataID = (uWAVE_DataID_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 3:
        if (ndIdx >= stIdx)
        {
          rdata->isValue = true;
          rdata->dataValue = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;

      case 4:
        if (ndIdx >= stIdx)
        {
          rdata->isPTime = true;
          rdata->pTime = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;

      case 5:
        if (ndIdx >= stIdx)
        {
          rdata->isAzimuth = true;
          rdata->azimuth = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_AQPNG_SETTINGS(uWAVE_AQPNG_SETTINGS_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWVO,[isSaveInFlash],AQPN_ModeID,[periodMs],[rcCmdID],[rcTxID],[rcRxID],[isPT],[pt_targetAddr]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  rdata->isSaveInFlash = false;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 2:
        if (ndIdx > stIdx)
          rdata->aqpng_Mode = (uWAVE_AQPNG_Mode_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        else
          result = false;
        break;

      case 3:
        if (ndIdx > stIdx)
        {
          rdata->isPeriod = true;
          rdata->periodMs = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        }
        break;

      case 4:
        if (ndIdx > stIdx)
        {
          rdata->isDataID = true;
          rdata->dataID = (uWAVE_DataID_Enum)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        }
        break;

      case 5:
        if (ndIdx > stIdx)
          rdata->rcTxID = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 6:
        if (ndIdx > stIdx)
          rdata->rcRxID = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 7:
        if (ndIdx > stIdx)
          rdata->isPT = (bool)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 8:
        if (ndIdx > stIdx)
        {
          rdata->pt_targetAddr = (byte)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        }
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}

bool uWAVE_Parse_DINFO(uWAVE_DINFO_Struct* rdata, const byte* buffer, byte idx)
{
  // $PUWV!,serialNumber,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,totalCh,salinityPSU,isPTS,isCmdMode
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          UCNL_STR_ReadString(buffer, rdata->serialNumber, &(rdata->serialNumber_size), stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          UCNL_STR_ReadString(buffer, rdata->core_moniker, &(rdata->sys_moniker_size), stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->core_version = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 4:
        if (ndIdx < stIdx)
          result = false;
        else
          UCNL_STR_ReadString(buffer, rdata->sys_moniker, &(rdata->sys_moniker_size), stIdx, ndIdx);
        break;
      case 5:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->sys_version = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 6:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->acBaudrate = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 7:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->txChID = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 8:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->rxChID = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 9:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->totalChannels = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 10:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->styPSU = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 11:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->isPTSPresent = (bool)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 12:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->isCmdMode = (bool)UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      default:
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  return result;
}


// Sentence builders
void uWAVE_Build_SETTINGS_WRITE(uWAVE_SETTINGS_WRITE_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWV1,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->rxChID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->txChID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteFloat(  buffer, idx, sdata->styPSU, 0, 1);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isCmdMode, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isACKOnTXFinished, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteFloat(  buffer, idx, sdata->gravityAcc, 0, 4);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_RC_REQUEST(uWAVE_RC_REQUEST_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWV2,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->txChID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->rxChID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->rcCmdID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_AMB_DTA_CFG(uWAVE_AMB_DTA_CFG_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWV6,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isSaveInFlash, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->periodMs, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isPrs, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isTemp, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isDpt, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isBatV, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_INC_DTA_CFG(uWAVE_INC_DTA_CFG_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWV8,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isSaveInFlash, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->periodMs, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_PT_SETTINGS_READ(byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVD,0*5C\r\n\0");
}

void uWAVE_Build_PT_SETTINGS_WRITE(uWAVE_PT_SETTINGS_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVF,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isSaveInFlash, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->isPtEnabled, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->ptAddress, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_PT_SEND(uWAVE_PT_PACKET_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVG,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->ptAddress, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->tries, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->ptAddress, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteHexStr( buffer, idx, sdata->dataPacket, sdata->dataPacketSize);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_PT_ABORT_SEND(byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVG,0,0,*6F\r\n\0");
}

void uWAVE_Build_PT_ITG(uWAVE_PT_ITG_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVK,\0");
  UCNL_STR_WriteIntDec( buffer, idx, sdata->ptAddress, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);
  UCNL_STR_WriteIntDec( buffer, idx, sdata->pt_itg_dataID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);

  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_AQPNG_SETTINGS_READ(byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVN,0*56\r\n\0");
}

void uWAVE_Build_AQPNG_SETTINGS(uWAVE_AQPNG_SETTINGS_Struct* sdata, byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWVO,\0");

  UCNL_STR_WriteIntDec( buffer, idx, sdata->isSaveInFlash, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  UCNL_STR_WriteIntDec( buffer, idx, sdata->aqpng_Mode, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  if (sdata->isPeriod)
    UCNL_STR_WriteIntDec( buffer, idx, sdata->periodMs, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  if (sdata->isDataID)
    UCNL_STR_WriteIntDec( buffer, idx, sdata->dataID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  UCNL_STR_WriteIntDec( buffer, idx, sdata->rcTxID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  UCNL_STR_WriteIntDec( buffer, idx, sdata->rcRxID, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  UCNL_STR_WriteIntDec( buffer, idx, sdata->isPT, 0);
  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_PAR_SEP);

  UCNL_STR_WriteIntDec( buffer, idx, sdata->pt_targetAddr, 0);

  UCNL_STR_WriteByte(   buffer, idx, UCNL_NMEA_CHK_SEP);
  UCNL_STR_WriteHexByte(buffer, idx, 0);
  UCNL_STR_WriteStr(    buffer, idx, (byte*)"\r\n");

  UCNL_NMEA_CheckSum_Update(buffer, *idx);
}

void uWAVE_Build_DINFO_GET(byte* buffer, byte bufferSize, byte* idx)
{
  UCNL_STR_WriterInit(  buffer, idx, bufferSize);
  UCNL_STR_WriteStr(    buffer, idx, "$PUWV?,0*27\r\n\0");
}
