/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#ifndef _UCNL_NAV_
#define _UCNL_NAV_

#define _PI                   (3.1415926535897932384626433832795)
#define PI2                   (_PI * 2.0)
#define PI_DBY_180            (_PI / 180.0)
#define D180_DBY_PI           (180.0 / _PI)

#define WGS84_MJ_SEMIAXIS_M   (6378137)
#define WGS84_IN_FLATTENING   (298.257223563)
#define WGS84_FLATTENING      (1.0 / WGS84_IN_FLATTENING)
#define WGS84_MN_SEMIAXIS_M   (WGS84_MJ_SEMIAXIS_M * (1 - WGS84_FLATTENING))
#define WGS84_ECCENTRICITY    (((WGS84_MJ_SEMIAXIS_M * WGS84_MJ_SEMIAXIS_M) - (WGS84_MN_SEMIAXIS_M * WGS84_MN_SEMIAXIS_M)) / (WGS84_MJ_SEMIAXIS_M * WGS84_MJ_SEMIAXIS_M))
#define WGS84_ECCENTRICITY_SQ (WGS84_ECCENTRICITY * WGS84_ECCENTRICITY)

#define UCNL_NAV_DEG2RAD(val) ((val) * PI_DBY_180)
#define UCNL_NAV_RAD2DEG(val) ((val) * D180_DBY_PI)



float UCNL_NAV_Wrap(float val, float lim);
float UCNL_NAV_Wrap2PI(float angle_rad);
float UCNL_NAV_Wrap360(float angle_deg);
float UCNL_NAV_Dist3D(float x1, float y1, float z1, float x2, float y2, float z2);
float UCNL_NAV_Dist2D(float x1, float y1, float x2, float y2);

void UCNL_NAV_PointOffset_WGS84(float lat_rad, float lon_rad, float lat_offset_m, float lon_offset_m, float* e_lat_rad,  float* e_lon_rad);
void UCNL_NAV_GetDeltasByGeopoints_WGS84(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad, float* delta_lat_m, float* delta_lon_m);

float UCNL_NAV_HaversineInverse(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad);
void UCNL_NAV_HaversineDirect(float sp_lat_rad, float sp_lon_rad, float dst_m, float fwd_az_rad, float* ep_lat_rad, float* ep_lon_rad);
float UCNL_NAV_HaversineInitialBearing(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad);
float UCNL_NAV_HaversineFinalBearing(float sp_lat_rad, float sp_lon_rad, float ep_lat_rad, float ep_lon_rad);

bool UCNL_NAV_CirclesIntersection(float x1, float y1, float r1, float x2, float y2, float r2, float* ix1, float* iy1, float* ix2, float* iy2);


#endif
