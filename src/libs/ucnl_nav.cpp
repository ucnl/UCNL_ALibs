/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#include "Arduino.h"
#include "ucnl_nav.h"

float UCNL_NAV_Wrap(float val, float lim)
{
  float sign = 1, vl = val;

  if (vl < 0)
  {
    vl = -vl;
    sign = -1;
  }

  while (vl > lim)
    vl -= lim;

  return vl * sign;
}

float UCNL_NAV_Wrap2PI(float angle_rad)
{
  return UCNL_NAV_Wrap(angle_rad, PI2);
}

float UCNL_NAV_Wrap360(float angle_deg)
{
  return UCNL_NAV_Wrap(angle_deg, 360);
}

float UCNL_NAV_Dist3D(float x1, float y1, float z1, float x2, float y2, float z2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

float UCNL_NAV_Dist2D(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/* Calculates location of a point by base point and latitudal and longitudal projections on WGS84 ellipsoid
   "lat_rad" base point latitude, ragians
   "lon_rad" base point longitude, ragians
   "lat_offset_m" latitudal offset in meters
   "lon_offset_m" longitudal offset in meters
   "e_lat_rad" new point latitude, radians
   "e_lon_rad" new point longitude, radians
*/
void UCNL_NAV_PointOffset_WGS84(float lat_rad, float lon_rad, float lat_offset_m, float lon_offset_m,
                                float* e_lat_rad,  float* e_lon_rad)
{
  float m_per_deg_lat = 111132.92 - 559.82 * cos(2.0 * lat_rad) + 1.175 * cos(4.0 * lat_rad);
  float m_per_deg_lon = 111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad);
  *e_lat_rad = lat_rad - PI_DBY_180 * lat_offset_m / m_per_deg_lat;
  *e_lon_rad = lon_rad - PI_DBY_180 * lon_offset_m / m_per_deg_lon;
}

/* Calculates latitudal and longitudal projections of a line on WGS84 ellipsoid between specified points
   "sp_lat_rad" start point latitude, ragians
   "sp_lon_rad" start point longitude, ragians
   "ep_lat_rad" end point latitude, ragians
   "ep_lon_rad" end point longitude, ragians
   "delta_lat_m" latitudal projection, meters
   "delta_lon_m" longitudal projection, meters
*/
void UCNL_NAV_GetDeltasByGeopoints_WGS84(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad,
    float* delta_lat_m, float* delta_lon_m)
{
  float m_lat_rad = (sp_lat_rad + ep_lat_rad) / 2.0;
  float m_per_deg_lat = 111132.92 - 559.82 * cos(2.0 * m_lat_rad) + 1.175 * cos(4.0 * m_lat_rad);
  float m_per_deg_lon = 111412.84 * cos(m_lat_rad) - 93.5 * cos(3.0 * m_lat_rad);
  *delta_lat_m = (sp_lat_rad - ep_lat_rad) * m_per_deg_lat * D180_DBY_PI;
  *delta_lon_m = (sp_lon_rad - ep_lon_rad) * m_per_deg_lon * D180_DBY_PI;
}

/* Solves inverse geodetic problem according to haversine equation
   "sp_lat_rad">start point latitude, radians
   "sp_lon_rad">start point longitude, radians
   "ep_lat_rad">end point latitude, radians
   "ep_lon_rad">end point longitude, radians
   resturns distance between specified points
*/
float UCNL_NAV_HaversineInverse(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad)
{
  float dLat = ep_lat_rad - sp_lat_rad;
  float dLon = ep_lon_rad - sp_lon_rad;
  float a = pow(sin(dLat / 2), 2) +
            cos(sp_lat_rad) * cos(ep_lat_rad) *
            sin(dLon / 2) * sin(dLon / 2);
  return WGS84_MJ_SEMIAXIS_M * 2 * atan2(sqrt(a), sqrt(1 - a));
}

/* Solves direct geodetic problem according to haversine equation
   "sp_lat_rad">start point latitude, radians
   "sp_lon_rad">start point longitude, radians
   "dst_m">distance on sphere, meters
   "fwd_az_rad">forward azimuth, radians
   "ep_lat_rad">end point latitude, radians
   "ep_lon_rad">end point longitude, radians
*/
void UCNL_NAV_HaversineDirect(float sp_lat_rad, float sp_lon_rad, float dst_m, float fwd_az_rad,
                              float* ep_lat_rad, float* ep_lon_rad)
{
  float delta = dst_m / WGS84_MJ_SEMIAXIS_M;
  *ep_lat_rad = asin(sin(sp_lat_rad) * cos(delta) + cos(sp_lat_rad) * sin(delta) * cos(fwd_az_rad));
  *ep_lon_rad = UCNL_NAV_Wrap2PI(3 * _PI + (sp_lon_rad + atan2(sin(fwd_az_rad) * sin(delta) * cos(sp_lat_rad),
                                 cos(delta) - sin(sp_lat_rad) * sin(*ep_lat_rad)))) - _PI;
}

/* Calculates initial bearing (forward azimuth) to a point
   "sp_lat_rad" start point latitude, radians
   "sp_lon_rad" start point longitude, radians
   "ep_lat_rad" end point latitude, radians
   "ep_lon_rad" end point longitude, radians
   returns initial bearing from start to end point
*/
float UCNL_NAV_HaversineInitialBearing(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad)
{
  float y = sin(ep_lon_rad - sp_lon_rad) * cos(ep_lat_rad);
  float x = cos(sp_lat_rad) * sin(ep_lat_rad) - sin(sp_lat_rad) * cos(ep_lat_rad) * cos(ep_lon_rad - sp_lon_rad);
  return UCNL_NAV_Wrap2PI(_PI + atan2(y, x));
}

/* Calculates final bearing (reverse azimuth) to a point
   "sp_lat_rad" start point latitude, radians
   "sp_lon_rad" start point longitude, radians
   "ep_lat_rad" end point latitude, radians
   "ep_lon_rad" end point longitude, radians
   returns initial bearing from start to end point
*/
float UCNL_NAV_HaversineFinalBearing(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad)
{
  return UCNL_NAV_Wrap2PI(UCNL_NAV_HaversineInitialBearing(ep_lat_rad, ep_lon_rad, sp_lat_rad, sp_lon_rad) + _PI);
}


bool UCNL_NAV_CirclesIntersection(float x1, float y1, float r1, float x2, float y2, float r2, float* ix1, float* iy1, float* ix2, float* iy2)
{
  float x2_1 = x2 - x1;
  float y2_1 = y2 - y1;
  float d = sqrt(x2_1 * x2_1 + y2_1 * y2_1);
  
  if ((d > abs(r1 - r2)) && (d < (r1 + r2))) // two points of intesection
  {
    float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    float h = sqrt(r1 * r1 - a * a);
    float x_t = x1 + a * x2_1 / d;
    float y_t = y1 + a * y2_1 / d;

    *ix1 = x_t + h * y2_1 / d;
    *iy1 = y_t - h * x2_1 / d;
    *ix2 = x_t - h * y2_1 / d;
    *iy2 = y_t + h * x2_1 / d;

    return true;
  }
  else
    return false;
}
