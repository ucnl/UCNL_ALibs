/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#include "Arduino.h"
#include "ucnl_str.h"
#include "ucnl_nmea.h"


void UCNL_NMEA_InitStruct(UCNL_NMEA_State_Struct* uState, byte* buffer, byte buffer_size, long* sntIDs, byte sntIDs_size)
{
  uState->isReady = false;
  uState->buffer = buffer;
  uState->buffer_size = buffer_size;
  uState->sntIDs = sntIDs;
  uState->sntIDs_size = sntIDs_size;
}

void UCNL_NMEA_Release(UCNL_NMEA_State_Struct* uState)
{
  uState->isReady = false;
}

UCNL_NMEA_Result_Enum UCNL_NMEA_Process_Byte(UCNL_NMEA_State_Struct* uState, byte newByte)
{
  UCNL_NMEA_Result_Enum result = UCNL_NMEA_RESULT_BYPASS_BYTE;

  if (!uState->isReady)
  {
    if (newByte == UCNL_NMEA_SNT_STR)
    {
      uState->isStarted = true;
      result = UCNL_NMEA_RESULT_PACKET_STARTED;

      for (int i = 0; i < uState->buffer_size; i++)
        uState->buffer[i] = 0;

      uState->chk_act     = 0;
      uState->chk_dcl     = 0;
      uState->chk_dcl_idx = 0;
      uState->idx         = 0;

      uState->tkrID       = 0;
      uState->sntID       = 0;

      uState->buffer[uState->idx] = newByte;
      uState->idx++;
    }
    else
    {
      if (uState->isStarted)
      {
        result = UCNL_NMEA_RESULT_PACKET_PROCESS;
        uState->buffer[uState->idx] = newByte;

        if (newByte == UCNL_NMEA_SNT_END)
        {
          uState->isStarted = false;
          uState->isReady = true;
          result = UCNL_NMEA_RESULT_PACKET_READY;
        }
        else if (newByte == UCNL_NMEA_CHK_SEP)
        {
          uState->chk_dcl_idx = 1;
          uState->chk_present = true;
        }
        else
        {
          if (uState->idx >= uState->buffer_size)
          {
            uState->isStarted = false;
            result = UCNL_NMEA_RESULT_PACKET_TOO_BIG;
          }
          else
          {
            if (uState->chk_dcl_idx == 0)
            {
              uState->chk_act ^= newByte;
              if      (uState->idx == 1)
                if (newByte == UCNL_NMEA_PSENTENCE_SYMBOL)
                  uState->isPSentence = true;
                else
                  uState->tkrID = ((int)newByte) << 8;
              else if (uState->idx == 2)
                if (uState->isPSentence)
                  uState->sntID = ((long)newByte) << 24;
                else
                  uState->tkrID |= newByte;
              else if (uState->idx == 3)
                if (uState->isPSentence)
                  uState->sntID |= (((long)newByte) << 16);
                else
                  uState->sntID = (((long)newByte) << 16);
              else if (uState->idx == 4)
                uState->sntID |= (((long)newByte) << 8);
              else if (uState->idx == 5)
              {
                uState->sntID |= newByte;

                byte i = 0;
                while ((i < uState->sntIDs_size) && (uState->sntID != uState->sntIDs[i]))
                  i++;

                if (i >= uState->sntIDs_size)
                {
                  uState->isStarted = false;
                  result = UCNL_NMEA_RESULT_PACKET_SKIPPING;
                }
              }
            }
            else if (uState->chk_dcl_idx == 1)
            {
              uState->chk_dcl = 16 * UCNL_STR_HEXDIGIT2B(newByte);
              uState->chk_dcl_idx++;
            }
            else if (uState->chk_dcl_idx == 2)
            {
              uState->chk_dcl += UCNL_STR_HEXDIGIT2B(newByte);
              if (uState->chk_act != uState->chk_dcl)
              {
                uState->isStarted = false;
                result = UCNL_NMEA_RESULT_PACKET_CHECKSUM_ERROR;
              }
              uState->chk_dcl_idx++;
            }
          }
        }
        uState->idx++;
      }
    }
  }
  return result;
}

bool UCNL_NMEA_Get_NextParam(const byte* buffer, byte fromIdx, byte size, byte* stIdx, byte* ndIdx)
{
  byte i = fromIdx + 1;
  *stIdx = fromIdx;
  *ndIdx = *stIdx;

  while ((i <= size) && (*ndIdx == *stIdx))
  {
    if ((buffer[i] == UCNL_NMEA_PAR_SEP) ||
        (buffer[i] == UCNL_NMEA_CHK_SEP) ||
        (buffer[i] == UCNL_NMEA_SNT_END1) ||
        (i == size))
    {
      *ndIdx = i;
    }
    else
    {
      i++;
    }
  }
  (*stIdx)++;
  (*ndIdx)--;

  return ((buffer[i] != UCNL_NMEA_CHK_SEP) && (i != size) && (buffer[i] != UCNL_NMEA_SNT_END1));
}

void UCNL_NMEA_CheckSum_Update(byte* buffer, byte size)
{
  byte i;
  byte acc = 0, b1, b2;
  for (i = 0; i < size; i++)
  {
    if      (buffer[i] == UCNL_NMEA_SNT_STR)
      acc = 0;
    else if (buffer[i] == UCNL_NMEA_CHK_SEP)
    {
      buffer[i + 1] = UCNL_STR_DIGIT_2HEX(acc / 16);
      buffer[i + 2] = UCNL_STR_DIGIT_2HEX(acc % 16);
    }
    else
    {
      acc ^= buffer[i];
    }
  }
}

bool UCNL_NMEA_Parse_RMC(UCNL_NMEA_RMC_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // Sentence example:
  // $GPRMC,230540.00,A,5312.1329616,N,15942.6950884,E,4.9,217.1,290421,999.9,E,D*3C

  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1: // Time
        if (ndIdx < stIdx)
          result = false;
        else
        {
          rdata->hour = UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]);
          rdata->minute = UCNL_STR_CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
          rdata->second = UCNL_STR_ParseFloat(buffer, stIdx + 4, ndIdx);

          if (!UCNL_NMEA_IS_VALID_HOUR(rdata->hour) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->minute) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->second))
            result = false;
        }
        break;
      case 2: // Time validity flag
        if ((ndIdx < stIdx) || (buffer[stIdx] != UCNL_NMEA_TD_VALID))
          result = false;
        break;
      case 3: // Latitude
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->latitude_deg = (float)UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]) +
                                UCNL_STR_ParseFloat(buffer, stIdx + 2, ndIdx) / 60.0;

        if (!UCNL_NMEA_IS_VALID_LATDEG(rdata->latitude_deg))
          result = false;
        break;
      case 4: // Latitude hemisphere
        if (ndIdx < stIdx)
          result = false;
        else if (buffer[stIdx] == UCNL_NMEA_SOUTH_SIGN)
          rdata->latitude_deg = -rdata->latitude_deg;
        break;
      case 5: // Longitude
        if (ndIdx <= stIdx)
          result = false;
        else
          rdata->longitude_deg = (float)UCNL_STR_CCC2B(buffer[stIdx], buffer[stIdx + 1], buffer[stIdx + 2]) +
                                 UCNL_STR_ParseFloat(buffer, stIdx + 3, ndIdx) / 60.0;

        if (!UCNL_NMEA_IS_VALID_LONDEG(rdata->longitude_deg))
          result = false;

        break;
      case 6: // Longitude hemisphere
        if (ndIdx < stIdx)
          result = false;
        else if (buffer[stIdx] == UCNL_NMEA_WEST_SIGN)
          rdata->longitude_deg = -rdata->longitude_deg;
        break;
      case 7: // Speed in knots
        if (ndIdx >= stIdx)
          rdata->speed_kmh = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx) * 1.852; //
        break;
      case 8: // Course in degrees
        if (ndIdx >= stIdx)
          rdata->course_deg = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 9: // Date
        if (ndIdx < stIdx)
          result = false;
        else
        {
          rdata->date = UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]);
          rdata->month = UCNL_STR_CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
          rdata->year = UCNL_STR_CC2B(buffer[stIdx + 4], buffer[stIdx + 5]);
          if (!UCNL_NMEA_IS_VALID_DATE(rdata->date) ||
              !UCNL_NMEA_IS_VALID_MONTH(rdata->month) ||
              !UCNL_NMEA_IS_VALID_YEAR(rdata->year))
            result = false;
        }
        break;
      case 12: // Data validity flag
        if ((ndIdx < stIdx) || (buffer[stIdx] == UCNL_NMEA_DATA_NOT_VALID))
          result = false;
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_GGA(UCNL_NMEA_GGA_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // $GNGGA,054157.013,2307.1261,N,12016.4308, E,1, 6,1.93,34.9,    M,17.8,M,,*76
  // $GPGGA,025346.726,5311.9987,N,15942.6717, E,1,04,    ,-11.4228,M,    ,M,,*49
  // $GPGGA,123143.00,4831.45878,N,04430.24139,E,1,05,3.68,19.0,    M,1.8, M,,*55

  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1: // Time
        if (ndIdx < stIdx)
          result = false;
        else
        {
          rdata->hour = UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]);
          rdata->minute = UCNL_STR_CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
          rdata->second = UCNL_STR_ParseFloat(buffer, stIdx + 4, ndIdx);

          if (!UCNL_NMEA_IS_VALID_HOUR(rdata->hour) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->minute) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->second))
            result = false;
        }
        break;
      case 2: // Latitude
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->latitude_deg = (float)UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]) +
                                UCNL_STR_ParseFloat(buffer, stIdx + 2, ndIdx) / 60.0;

        if (!UCNL_NMEA_IS_VALID_LATDEG(rdata->latitude_deg))
          result = false;
        break;
      case 3: // Latitude hemisphere
        if (ndIdx < stIdx)
          result = false;
        else if (buffer[stIdx] == UCNL_NMEA_SOUTH_SIGN)
          rdata->latitude_deg = -rdata->latitude_deg;
        break;
      case 4: // Longitude
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->longitude_deg = (float)UCNL_STR_CCC2B(buffer[stIdx], buffer[stIdx + 1], buffer[stIdx + 2]) +
                                 UCNL_STR_ParseFloat(buffer, stIdx + 3, ndIdx) / 60.0;

        if (!UCNL_NMEA_IS_VALID_LONDEG(rdata->longitude_deg))
          result = false;
        break;
      case 5: // Longitude hemisphere
        if (ndIdx < stIdx)
          result = false;
        else if (buffer[stIdx] == UCNL_NMEA_WEST_SIGN)
          rdata->longitude_deg = -rdata->longitude_deg;
        break;
      case 6:
        // GNSS quality indicator
        if (ndIdx >= stIdx)
          rdata->gnss_qly_ind = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 7:
        // satellites in use
        if (ndIdx >= stIdx)
          rdata->sats_in_use = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 8:
        // HDOP
        if (ndIdx >= stIdx)
          rdata->hdop = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 9:
        // orthometricHeight
        if (ndIdx >= stIdx)
          rdata->orth_height = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      //case 10:
      // orthometricHeight units
      //     break;
      case 11:
        // geoidal separation
        if (ndIdx >= stIdx)
          rdata->gsep = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      //case 12:
      // geoidal separation units
      //     break;
      case 13:
        // dgps_rec_age
        if (ndIdx >= stIdx)
          rdata->dgps_rec_age = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 14:
        // datum id
        if (ndIdx >= stIdx)
          rdata->datumID = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_GLL(UCNL_NMEA_GLL_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // $GPGLL,4831.45336,N,04430.22910,E,131116.00,A,A*6E
  // $GNGLL,4831.4600,N,04430.2750,E,125414.000,A,A*4F

  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1: // Latitude
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->latitude_deg = (float)UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]) +
                                UCNL_STR_ParseFloat(buffer, stIdx + 2, ndIdx) / 60.0;
        break;
      case 2: // Latitude hemisphere
        if (ndIdx < stIdx)
          result = false;
        else if (buffer[stIdx] == UCNL_NMEA_SOUTH_SIGN)
          rdata->latitude_deg = -rdata->latitude_deg;
        break;
      case 3: // Longitude
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->longitude_deg = (float)UCNL_STR_CCC2B(buffer[stIdx], buffer[stIdx + 1], buffer[stIdx + 2]) +
                                 UCNL_STR_ParseFloat(buffer, stIdx + 3, ndIdx) / 60.0;
        break;
      case 4: // Longitude hemisphere
        if (ndIdx < stIdx)
          result = false;
        else if (buffer[stIdx] == UCNL_NMEA_WEST_SIGN)
          rdata->longitude_deg = -rdata->longitude_deg;
        break;
      case 5: // Time
        if (ndIdx < stIdx)
          result = false;
        else
        {
          rdata->hour = UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]);
          rdata->minute = UCNL_STR_CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
          rdata->second = UCNL_STR_ParseFloat(buffer, stIdx + 4, ndIdx);

          if (!UCNL_NMEA_IS_VALID_HOUR(rdata->hour) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->minute) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->second))
            result = false;
        }
        break;
      case 6:
        if ((ndIdx < stIdx) || (buffer[stIdx] == UCNL_NMEA_DATA_NOT_VALID))
          result = false;
        break;
      case 7:
        if ((ndIdx < stIdx) || (buffer[stIdx] == UCNL_NMEA_PMODE_DATA_NOT_VALID))
          result = false;
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_VTG(UCNL_NMEA_VTG_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // $GPVTG,,T,,M,0.338,N,0.627,K,A*28
  // $GPVTG,59.58,T,,M,1.43,N,2.65,K,A*0B


  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1: // trackTrue
        if (ndIdx >= stIdx)
          rdata->track_true_deg = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        else
          result = false;
        break;
      case 3: // trackMagnetic
        if (ndIdx >= stIdx)
        {
          rdata->is_magnetic = true;
          rdata->track_magnetic_deg = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        break;
      case 5: // speedKnots
        if (ndIdx >= stIdx)
          rdata->speed_knots = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 7: // speedKmh
        if (ndIdx >= stIdx)
          rdata->speed_kmh = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 8:
        if ((ndIdx < stIdx) || (buffer[stIdx] == UCNL_NMEA_DATA_NOT_VALID))
          result = false;
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_HDT(UCNL_NMEA_HDT_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  // $GPHDT,253.423,T*34

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1:
        if (ndIdx >= stIdx)
          rdata->track_true_deg = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        else
          result = false;
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_HDG(UCNL_NMEA_HDG_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  // $GPHDG,253.423,,,*34

  float magnetic_heading_deg;
  bool is_magnetic_variation;
  float magnetic_variation;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1:
        if (ndIdx >= stIdx)
          rdata->magnetic_heading_deg = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        else
          result = false;
        break;
      case 3:
        if (ndIdx >= stIdx)
        {
          rdata->is_magnetic_variation = true;
          rdata->magnetic_variation = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        }
        else
          rdata->is_magnetic_variation = false;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_MTW(UCNL_NMEA_MTW_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  // $GPMTW,253.423,C*34

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1:
        if (ndIdx >= stIdx)
          rdata->mean_water_temperature_c = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        else
          result = false;
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_GSA(UCNL_NMEA_GSA_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // $GPGSA,A,3,18,26,23,16,27,10,,,,,,,3.51,1.83,2.99*02

  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1: // Mode
        if (ndIdx >= stIdx)
          rdata->mode = buffer[stIdx];
        break;
      case 2: // fixType
        if (ndIdx >= stIdx)
          rdata->fix_type = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 3: // PRNs
      case 4:
      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
        if (ndIdx >= stIdx)
          rdata->prns[pIdx - 3] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        else
          rdata->prns[pIdx - 3] = 0;
        break;
      case 15: // PDOP
        if (ndIdx >= stIdx)
          rdata->pdop = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 16:
        if (ndIdx >= stIdx)
          rdata->hdop = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 17:
        if (ndIdx >= stIdx)
          rdata->vdop = UCNL_STR_ParseFloat(buffer, stIdx, ndIdx);
        break;
      case 18:
        rdata->systemID = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_GSV(UCNL_NMEA_GSV_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  // $GPGSV,3,1,12,08,15,313,15,10,56,213,35,13,14,037,14,15,31,064,20*75
  // $GPGSV,3,2,12,16,35,250,31,18,56,078,21,20,03,055,,23,81,113,18*7A
  // $GPGSV,3,3,12,26,19,225,26,27,47,303,18,29,10,140,08,32,00,176,*7E

  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  rdata->satDataNum = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1:
        if (ndIdx >= stIdx)
          rdata->total_messages = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        else
          result = false;
        break;

      case 2:
        if (ndIdx >= stIdx)
          rdata->current_message = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        else
          result = false;
        break;

      case 3:
        if (ndIdx >= stIdx)
          rdata->sats_in_view = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        else
          result = false;
        break;

// Sat 1 section
      case 4:
        if (ndIdx >= stIdx)
          rdata->PRNNumbers[0] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 5:
        if (ndIdx >= stIdx)
          rdata->Elevations[0] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 6:
        if (ndIdx >= stIdx)
          rdata->Azimuths[0] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 7:
        if (ndIdx >= stIdx)
        {
          rdata->SNRs[0] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
          rdata->satDataNum = 1;
        }
        break;
        
// Sat 2 section
      case 8:
        if (ndIdx >= stIdx)
          rdata->PRNNumbers[1] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 9:
        if (ndIdx >= stIdx)
          rdata->Elevations[1] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 10:
        if (ndIdx >= stIdx)
          rdata->Azimuths[1] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 11:
        if (ndIdx >= stIdx)
        {
          rdata->SNRs[1] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
          rdata->satDataNum = 2;
        }

// Sat 3 section
      case 12:
        if (ndIdx >= stIdx)
          rdata->PRNNumbers[2] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 13:
        if (ndIdx >= stIdx)
          rdata->Elevations[2] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 14:
        if (ndIdx >= stIdx)
          rdata->Azimuths[2] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 15:
        if (ndIdx >= stIdx)
        {
          rdata->SNRs[2] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
          rdata->satDataNum = 3;
        }

// Sat 4 section
      case 16:
        if (ndIdx >= stIdx)
          rdata->PRNNumbers[3] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 17:
        if (ndIdx >= stIdx)
          rdata->Elevations[3] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 18:
        if (ndIdx >= stIdx)
          rdata->Azimuths[3] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;

      case 19:
        if (ndIdx >= stIdx)
        {
          rdata->SNRs[3] = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
          rdata->satDataNum = 4;
        }

      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result); 

  rdata->isValid = result;
  return result;
}

bool UCNL_NMEA_Parse_ZDA(UCNL_NMEA_ZDA_RESULT_Struct* rdata, const byte* buffer, byte idx)
{
  bool isNotLastParam = false;
  bool result = true;
  byte pIdx = 0, ndIdx = 0, stIdx = 0;

  do
  {
    isNotLastParam = UCNL_NMEA_Get_NextParam(buffer, ndIdx + 1, idx, &stIdx, &ndIdx);

    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
        {
          rdata->hour = UCNL_STR_CC2B(buffer[stIdx], buffer[stIdx + 1]);
          rdata->minute = UCNL_STR_CC2B(buffer[stIdx + 2], buffer[stIdx + 3]);
          rdata->second = UCNL_STR_ParseFloat(buffer, stIdx + 4, ndIdx);

          if (!UCNL_NMEA_IS_VALID_HOUR(rdata->hour) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->minute) ||
              !UCNL_NMEA_IS_VALID_MINSEC(rdata->second))
            result = false;
        }
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->day = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->month = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 4:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->year = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 5:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->t_zone_offset_hours = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      case 6:
        if (ndIdx < stIdx)
          result = false;
        else
          rdata->t_zone_offset_minutes = UCNL_STR_ParseIntDec(buffer, stIdx, ndIdx);
        break;
      default:
        break;
    }
    pIdx++;
  } while (isNotLastParam && result);

  rdata->isValid = result;
  return result;
}
