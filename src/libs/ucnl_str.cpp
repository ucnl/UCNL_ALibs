/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#include "Arduino.h"
#include "ucnl_str.h"

void UCNL_STR_WriterInit(byte* buffer, byte* srcIdx, byte bufferSize)
{
  *srcIdx = 0;
  for (int i = 0; i < bufferSize; i++)
    buffer[i] = 0;
}

void UCNL_STR_WriteByte(byte* buffer, byte* srcIdx, byte c)
{
  buffer[*srcIdx] = c;
  (*srcIdx)++;
}

void UCNL_STR_WriteIntDec(byte* buffer, byte* srcIdx, long src, byte zPad)
{
  long x = src;
  int len = 0, i;

  do {
    x /= 10;
    len++;
  } while (x >= 1);

  x = 1;
  for (i = 1; i < len; i++) x *= 10;

  if (zPad > 0) i = zPad;
  else i = len;

  do
  {
    if (i > len) buffer[*srcIdx] = '0';
    else
    {
      buffer[*srcIdx] = (byte)((src / x) + '0');
      src -= (src / x) * x;
      x /= 10;
    }
    (*srcIdx)++;
  } while (--i > 0);
}

void UCNL_STR_WriteFloat(byte* buffer, byte* srcIdx, float f, byte dPlaces, byte zPad)
{
  float ff = f;

  if (ff < 0)
  {
    UCNL_STR_WriteByte(buffer, srcIdx, '-');
    ff = -f;
  }

  long dec = (long)ff, mult = 1;
  int i;
  for (i = 0; i < dPlaces; i++) mult *= 10;
  long frac = (long)((ff - dec) * (float)mult);

  UCNL_STR_WriteIntDec(buffer, srcIdx, dec, zPad);
  UCNL_STR_WriteByte(buffer, srcIdx, '.');
  UCNL_STR_WriteIntDec(buffer, srcIdx, frac, dPlaces);
}

void UCNL_STR_WriteStr(byte* buffer, byte* srcIdx, byte* src)
{
  byte c;
  c = *src;
  while (c != '\0')
  {
    buffer[(*srcIdx)++] = c;
    c = *++src;
  }
}

void UCNL_STR_WriteHexByte(byte* buffer, byte* srcIdx, byte c)
{
  buffer[*srcIdx] = UCNL_STR_DIGIT_2HEX(c / 16);
  (*srcIdx)++;
  buffer[*srcIdx] = UCNL_STR_DIGIT_2HEX(c % 16);
  (*srcIdx)++;
}

void UCNL_STR_WriteHexArray(byte* buffer, byte* srcIdx, byte* src, byte srcSize)
{
  int i;
  UCNL_STR_WriteStr(buffer, srcIdx, UCNL_STR_HEX_ARRAY_PFX);
  for (i = 0; i < srcSize; i++)
    UCNL_STR_WriteHexByte(buffer, srcIdx, src[i]);
}

void UCNL_STR_WriteHexStr(byte* buffer, byte* srcIdx, byte* src, byte srcSize)
{
  int i;
  for (i = 0; i < srcSize; i++)
    UCNL_STR_WriteHexByte(buffer, srcIdx, src[i]);
}




float UCNL_STR_ParseFloat(const byte* buffer, byte stIdx, byte ndIdx)
{
  int i, dotIdx = ndIdx + 1;
  float sign = 1.0f, fract = 0.0f;

  if (buffer[stIdx] == '-')
  {
    sign = -1.0f;
    stIdx++;
  }

  for (i = stIdx; i <= ndIdx; i++)
  {
    if (buffer[i] == '.')
    {
      dotIdx = i;
    }
  }

  float result = 0.0f;
  float multiplier = 1.0f;

  for (i = dotIdx - 1; i >= stIdx; i--)
  {
    result += ((float)((buffer[i] - '0'))) * multiplier;
    multiplier *= 10.0f;
  }

  multiplier = 0.1f;

  for (i = dotIdx + 1; i <= ndIdx; i++)
  {
    fract += ((float)((buffer[i] - '0'))) * multiplier;
    multiplier /= 10.0f;
  }

  result += fract;
  return result * sign;
}

long UCNL_STR_ParseIntDec(const byte* buffer, byte stIdx, byte ndIdx)
{
  byte i;
  int sign = 1;

  if (buffer[stIdx] == '-')
  {
    sign = -1;
    stIdx++;
  }

  long result = 0;
  long multiplier = 1;

  for (i = ndIdx; i >= stIdx; i--)
  {
    result += ((int)((buffer[i] - '0'))) * multiplier;
    multiplier *= 10;
  }

  return result;
}

byte UCNL_STR_ParseHexByte(const byte* buffer, byte stIdx)
{
  return UCNL_STR_HEXDIGIT2B(buffer[stIdx]) * 16 + UCNL_STR_HEXDIGIT2B(buffer[stIdx + 1]);
}

int UCNL_STR_ReadHexStr(const byte* buffer, byte stIdx, byte ndIdx, byte* out_buffer, byte out_buffer_size, byte* out_size)
{
  int result = 0;
  for (int i = 0; i < out_buffer_size; i++)
    out_buffer[i] = 0;

  *out_size = (ndIdx - stIdx - 1);
  if ((*out_size < 0) || ((*out_size) % 2 != 0))
  {
    result = 1;
  }
  else
  {
    *out_size /= 2;
    if ((buffer[stIdx] != '0') || (buffer[stIdx + 1] != 'x'))
      result = 2;
    else
    {
      int idx = 0;
      while (idx < *out_size)
      {
        out_buffer[idx] = UCNL_STR_ParseHexByte(buffer, stIdx + idx * 2 + 2);
        idx++;
      }
    }
  }

  return result;
}

void UCNL_STR_ReadString(const byte* src_buffer, byte* dst_buffer, byte* bytesRead, byte stIdx, byte ndIdx)
{
  *bytesRead = 0;
  for (byte i = stIdx; i <= ndIdx; i++)
  {
    dst_buffer[i - stIdx] = src_buffer[i];
    (*bytesRead)++;
  }
}
