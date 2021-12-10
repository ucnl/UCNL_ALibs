/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#ifndef _UCNL_WPHX_
#define _UCNL_WPHX_

#define UCNL_WPHX_FWTR_DENSITY_KGM3        (998.02)       // Fresh water density at 20°C
#define UCNL_WPHX_FWTR_SOUND_SPEED_MPS     (1500.0)       //
#define UCNL_WPHX_FWTR_SALINITY_PSU        (0.0)          //
#define UCNL_WPHX_GRAVITY_ACC_MPS2         (9.80665)      // ISO 80000-3:2006
#define UCNL_WPHX_ATM_PRESSURE_MBAR        (1013.25)      // Average at sea level

/// calculates in situ density of water
/// millero et al 1980, deep-sea res.,27a,255-264
/// jpots ninth report 1978,tenth report 1980
float UCNL_WPHX_water_density_calc(float t, float p, float s);

/// The UNESCO equation: Chen and Millero (1977)
float UCNL_WPHX_speed_of_sound_UNESCO_calc(float t, float p, float s);

/// Calculates gravity at sea level vs latitude
/// WGS84 ellipsoid gravity formula
float UCNL_WPHX_gravity_constant_wgs84_calc(float phi);

/// calculates distance from the water surface where pressure is p0 to the point, where pressure is p
float UCNL_WPHX_depth_by_pressure_calc(float p, float p0, float rho, float g);

// Calculates pressure of a water column with given height (distance between
// the water surface and the given point) assuming constant water density
// h - depth, m
// p0 - atmospheric pressure, mBar
// rho - water density, kg/m^3
// g - gravity acceleration, m/s^2
float UCNL_WPHX_pressure_by_depth_calc(float h, float p0, float rho, float g);

// Calculated the freezing temperature of seawater (in °C) with specified pressure and salinity.
// According to:
// Algorithms for computation of fundamental properties of seawater.
// Unesco technical papers in marine science vol. 44, 1983, pp. 30
// https://darchive.mblwhoilibrary.org/bitstream/handle/1912/2470/059832eb.pdf
// p - pressure, mBar
// s - PSU
float UCNL_WPHX_water_fpoint_calc(float p, float s);

#endif
