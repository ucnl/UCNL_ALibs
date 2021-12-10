/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#ifndef _UCNL_VLBL_
#define _UCNL_VLBL_

#include "ucnl_nav.h"

#define VLBL_POINTS_RING_SIZE (3)
#define VLBL_HEAPS_RING_SIZE  (4)

typedef struct
{
  float xs[VLBL_POINTS_RING_SIZE];
  float ys[VLBL_POINTS_RING_SIZE];
  float ds[VLBL_POINTS_RING_SIZE];
  byte cnt;
  byte idx;  
} VLBL_Points_Ring_Struct;

typedef struct
{
  float lxs[VLBL_HEAPS_RING_SIZE];
  float lys[VLBL_HEAPS_RING_SIZE];
  float rxs[VLBL_HEAPS_RING_SIZE];
  float rys[VLBL_HEAPS_RING_SIZE];
  byte cnt;
  byte idx;
  float clx;
  float cly;
  float crx;
  float cry;
  float l_drms;
  float r_drms;
  float x;
  float y;
  float drms;
  float x_best;
  float y_best;
  float drms_best;
} VLBL_Ring_Struct;

void UCNL_VLBL_ResetStructs(VLBL_Points_Ring_Struct* vlblRing, VLBL_Ring_Struct* lrRing);
void UCNL_VLBL_Points_Ring_Reset(VLBL_Points_Ring_Struct* vlblRing);
void UCNL_VLBL_AddPoint(VLBL_Points_Ring_Struct* vlblRing, float x, float y, float d);
void UCNL_VLBL_Heap_UpdateCentroids(VLBL_Ring_Struct* lrRing);
void UCNL_VLBL_Heap_UpdateDRMSs(VLBL_Ring_Struct* lrRing);
void UCNL_VLBL_Heap_ProcessPoints(VLBL_Ring_Struct* lrRing, float x1, float y1, float x2, float y2);
void UCNL_VLBL_Clusterize(VLBL_Points_Ring_Struct* vlblRing, VLBL_Ring_Struct* lrRing, float x, float y, float d);

#endif
