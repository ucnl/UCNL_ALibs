/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#ifndef _UCNL_STR_
#define _UCNL_STR_

#define UCNL_STR_HEXDIGIT2B(b)     ((b) >= 0x41 ? ((b) - 0x37) : ((b) - 0x30))
#define UCNL_STR_DIGIT_2HEX(h)     ((h) > 9     ? ((h) + 0x37) : ((h) + 0x30))

#define UCNL_STR_C2B(b)            ((b - '0'))
#define UCNL_STR_CC2B(b1, b2)      ((10 * (b1 - '0') + (b2 - '0')))
#define UCNL_STR_CCC2B(b1, b2, b3) ((100 * (b1 - '0') + 10 * (b2 - '0') + (b3 - '0')))

#define UCNL_STR_HEX_ARRAY_PFX     ((byte*)"0x\0")

void UCNL_STR_WriterInit(byte* buffer, byte* srcIdx, byte bufferSize);
void UCNL_STR_WriteByte(byte* buffer, byte* srcIdx, byte c);
void UCNL_STR_WriteIntDec(byte* buffer, byte* srcIdx, long src, byte zPad);
void UCNL_STR_WriteFloat(byte* buffer, byte* srcIdx, float f, byte dPlaces, byte zPad);
void UCNL_STR_WriteHexByte(byte* buffer, byte* srcIdx, byte c);
void UCNL_STR_WriteHexArray(byte* buffer, byte* srcIdx, byte* src, byte srcSize);
void UCNL_STR_WriteHexStr(byte* buffer, byte* srcIdx, byte* src, byte srcSize);
void UCNL_STR_WriteStr(byte* buffer, byte* srcIdx, byte* src);

float UCNL_STR_ParseFloat(const byte* buffer, byte stIdx, byte ndIdx);
long  UCNL_STR_ParseIntDec(const byte* buffer, byte stIdx, byte ndIdx);
byte  UCNL_STR_ParseHexByte(const byte* buffer, byte stIdx);
int   UCNL_STR_ReadHexStr(const byte* buffer, byte stIdx, byte ndIdx, byte* out_buffer, byte out_buffer_size, byte* out_size);
void  UCNL_STR_ReadString(const byte* src_buffer, byte* dst_buffer, byte* bytesRead, byte stIdx, byte ndIdx);

#endif
