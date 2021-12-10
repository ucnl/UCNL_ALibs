/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#include "Arduino.h"
#include "ucnl_vlbl.h"

void UCNL_VLBL_Points_Ring_Reset(VLBL_Points_Ring_Struct* vlblRing)
{
  vlblRing->idx = 0;
  vlblRing->cnt = 0;
}

void UCNL_VLBL_ResetStructs(VLBL_Points_Ring_Struct* vlblRing, VLBL_Ring_Struct* lrRing)
{
  vlblRing->idx = 0;
  vlblRing->cnt = 0;

  lrRing->cnt = 0;
  lrRing->idx = 0;
  lrRing->drms_best = 10E+6;
}

void UCNL_VLBL_AddPoint(VLBL_Points_Ring_Struct* vlblRing, float x, float y, float d)
{
  vlblRing->xs[vlblRing->idx] = x;
  vlblRing->ys[vlblRing->idx] = y;
  vlblRing->ds[vlblRing->idx] = d;

  vlblRing->idx = (vlblRing->idx + 1) % VLBL_POINTS_RING_SIZE;
  if (vlblRing->cnt < VLBL_POINTS_RING_SIZE)
    vlblRing->cnt++;
}

void UCNL_VLBL_Heap_UpdateCentroids(VLBL_Ring_Struct* lrRing)
{
  if (lrRing->cnt <= 0) return;
  
  lrRing->clx = 0;
  lrRing->cly = 0;
  lrRing->crx = 0;
  lrRing->cry = 0;
  
  for (int n = 0; n < lrRing->cnt; n++)
  {
    lrRing->clx += lrRing->lxs[n];
    lrRing->cly += lrRing->lys[n];
    lrRing->crx += lrRing->rxs[n];
    lrRing->cry += lrRing->rys[n];
  }

  lrRing->clx /= lrRing->cnt;
  lrRing->cly /= lrRing->cnt;
  lrRing->crx /= lrRing->cnt;
  lrRing->cry /= lrRing->cnt;
}

void UCNL_VLBL_Heap_UpdateDRMSs(VLBL_Ring_Struct* lrRing)
{
  if (lrRing->cnt <= 0) return;

  float delta;
  float l_sigma_x = 0;
  float l_sigma_y = 0;
  float r_sigma_x = 0;
  float r_sigma_y = 0;
  
  for (int n = 0; n < lrRing->cnt; n++)
  {
    delta = lrRing->lxs[n] - lrRing->clx;
    l_sigma_x += delta * delta;

    delta = lrRing->lys[n] - lrRing->cly;
    l_sigma_y += delta * delta;

    delta = lrRing->rxs[n] - lrRing->crx;
    r_sigma_x += delta * delta;

    delta = lrRing->rys[n] - lrRing->cry;
    r_sigma_y += delta * delta;
  }

  l_sigma_x /= lrRing->cnt;
  l_sigma_y /= lrRing->cnt;
  r_sigma_x /= lrRing->cnt;
  r_sigma_y /= lrRing->cnt;

  lrRing->l_drms = sqrt(l_sigma_x * l_sigma_x + l_sigma_y * l_sigma_y);
  lrRing->r_drms = sqrt(r_sigma_x * r_sigma_x + r_sigma_y * r_sigma_y);
}

void UCNL_VLBL_Heap_ProcessPoints(VLBL_Ring_Struct* lrRing, float x1, float y1, float x2, float y2)
{
  if (lrRing->cnt == 0)
  {
    lrRing->lxs[0] = x1;
    lrRing->lys[0] = y1;
    lrRing->rxs[0] = x2;
    lrRing->rys[0] = y2;
    lrRing->idx = 1;
    lrRing->cnt = 1;
    lrRing->clx = x1;
    lrRing->cly = y1;
    lrRing->crx = x2;
    lrRing->cry = y2;
    lrRing->l_drms = -1;
    lrRing->r_drms = -1;

    lrRing->x = 0;
    lrRing->y = 0;
    lrRing->drms = 1E+6;
    
    lrRing->x_best = 0;
    lrRing->y_best = 0;
    lrRing->drms_best = 1E+6;
  }
  else
  {
    // find which point to which heap fits better
    float dst;
    float dst_min = UCNL_NAV_Dist2D(x1, y1, lrRing->clx, lrRing->cly);
    byte dst_min_idx = 0;
    
    dst = UCNL_NAV_Dist2D(x1, y1, lrRing->crx, lrRing->cry);
    if (dst < dst_min) { dst_min = dst; dst_min_idx = 1; }

    dst = UCNL_NAV_Dist2D(x2, y2, lrRing->clx, lrRing->cly);
    if (dst < dst_min) { dst_min = dst; dst_min_idx = 2; }
    
    dst = UCNL_NAV_Dist2D(x2, y2, lrRing->crx, lrRing->cry);
    if (dst < dst_min) { dst_min = dst; dst_min_idx = 3; }

    if ((dst_min_idx == 0) || (dst_min_idx == 3))
    {
      lrRing->lxs[lrRing->idx] = x1;
      lrRing->lys[lrRing->idx] = y1;
      lrRing->rxs[lrRing->idx] = x2;
      lrRing->rys[lrRing->idx] = y2;
    }
    else
    {
      lrRing->lxs[lrRing->idx] = x2;
      lrRing->lys[lrRing->idx] = y2;
      lrRing->rxs[lrRing->idx] = x1;
      lrRing->rys[lrRing->idx] = y1;
    }

    lrRing->idx = (lrRing->idx + 1) % VLBL_HEAPS_RING_SIZE;
    if (lrRing->cnt < VLBL_HEAPS_RING_SIZE) lrRing->cnt++;

    // recalculate centroids of both heaps
    UCNL_VLBL_Heap_UpdateCentroids(lrRing);

    // recalculate DRMS values of both heaps
    UCNL_VLBL_Heap_UpdateDRMSs(lrRing);
    
    if (lrRing->l_drms < lrRing->r_drms)
    {
      lrRing->x = lrRing->clx;
      lrRing->y = lrRing->cly;
      lrRing->drms = lrRing->l_drms;
    }
    else
    {
      lrRing->x = lrRing->crx;
      lrRing->y = lrRing->cry;
      lrRing->drms = lrRing->r_drms;
    }

    if (lrRing->drms < lrRing->drms_best)
    {
      lrRing->drms_best = lrRing->drms;
      lrRing->x_best = lrRing->x;
      lrRing->y_best = lrRing->y;      
    }
  }
}

void UCNL_VLBL_Clusterize(VLBL_Points_Ring_Struct* vlblRing, VLBL_Ring_Struct* lrRing, float x, float y, float d)
{
  float ix1, iy1, ix2, iy2;

  for (int n = 0; n < vlblRing->cnt; n++)
  {
    if (UCNL_NAV_CirclesIntersection(vlblRing->xs[n], vlblRing->ys[n], vlblRing->ds[n],
                                     x, y, d,
                                     &ix1, &iy1, &ix2, &iy2))
      {
        // ok, there are two points of intersection
        // put it to the heaps
        UCNL_VLBL_Heap_ProcessPoints(lrRing, ix1, iy1, ix2, iy2);
      }
  }
    
  UCNL_VLBL_AddPoint(vlblRing, x, y, d);
}
