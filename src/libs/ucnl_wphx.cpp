/*
Copyright (C) 2021, Underwater communication & navigation laboratory
All rights reserved.

www.unavlab.com
hello@unavlab.com

*/

#include "Arduino.h"
#include "ucnl_wphx.h"

/// calculates in situ density of water
/// millero et al 1980, deep-sea res.,27a,255-264
/// jpots ninth report 1978,tenth report 1980
float UCNL_WPHX_water_density_calc(float t, float p, float s)
{
  p = p / 1000.0;
  float sr = sqrt(s);
  float sig = (((4.8314E-4 * s) +
                ((-1.6546E-6 * t + 1.0227E-4) * t - 5.72466E-3) * sr +
                (((5.3875E-9 * t - 8.2467E-7) * t + 7.6438E-5) * t - 4.0899E-3) * t + 0.824493) * s) +
              ((((6.536332E-9 * t - 1.120083E-6) * t + 1.001685E-4) * t - 9.095290E-3) * t + 6.793952E-2) * t - 0.157406;

  float b = ((9.1697E-10 * t + 2.0816E-8) * t - 9.9348E-7) * s + (5.2787E-8 * t - 6.12293E-6) * t + 8.50935E-5;

  float k0 = (((((-5.3009E-4 * t + 1.6483E-2) * t + 7.944E-2) * sr) +
               ((-6.1670E-5 * t + 1.09987E-2) * t - 0.603459) * t + 54.6746) * s) +
             (((-5.155288E-5 * t + 1.360477E-2) * t - 2.327105) * t + 148.4206) * t + 19652.21;

  float a = (1.91075E-4 * sr + (-1.6078E-6 * t - 1.0981E-5) * t + 2.2838E-3) * s +
            ((-5.77905E-7 * t + 1.16092E-4) * t + 1.43713E-3) * t + 3.239908;

  float k = (b * p + a) * p + k0;

  return 1000.0f + (k * sig + 1000.0 * p) / (k - p);
}

/// The UNESCO equation: Chen and Millero (1977)
float UCNL_WPHX_speed_of_sound_UNESCO_calc(float t, float p, float s)
{
  p = p / 1000.0;
  float sr = sqrt(s);

  float d = 1.727E-3 - 7.9836E-6 * p;

  float b_1 = 7.3637E-5 + 1.7945E-7 * t;
  float b_0 = -1.922E-2 - 4.42E-5 * t;
  float b = b_0 + b_1 * p;

  float a_3 = (-3.389E-13 * t + 6.649E-12)  * t + 1.100E-10;
  float a_2 = ((7.988E-12 * t - 1.6002E-10) * t + 9.1041E-9) * t - 3.9064E-7;
  float a_1 = (((-2.0122E-10 * t + 1.0507E-8)  * t - 6.4885E-8) * t - 1.2580E-5) * t + 9.4742E-5;
  float a_0 = (((-3.21E-8 * t + 2.006E-6) * t + 7.164E-5) * t - 1.262E-2) * t + 1.389;
  float a = ((a_3 * p + a_2) * p + a_1) * p + a_0;

  float c_3 = (-2.3643E-12 * t + 3.8504E-10) * t - 9.7729E-9;
  float c_2 = (((1.0405E-12 * t - 2.5335E-10) * t + 2.5974E-8) * t - 1.7107E-6)  * t + 3.1260E-5;
  float c_1 = (((-6.1185E-10 * t + 1.3621E-7)  * t - 8.1788E-6) * t + 6.8982E-4)  * t + 0.153563;
  float c_0 = ((((3.1464E-9  * t - 1.47800E-6) * t + 3.3420E-4) * t - 5.80852E-2) * t + 5.03711) * t + 1402.388;
  float c  = ((c_3 * p + c_2) * p + c_1) * p + c_0;

  return c + (a + b * sr + d * s) * s;
}

/// Calculates gravity at sea level vs latitude
/// WGS84 ellipsoid gravity formula
float UCNL_WPHX_gravity_constant_wgs84_calc(float phi)
{
  float phi_sq = sin(phi);
  phi_sq *= phi_sq;
  return (9.7803253359 * ((1.0 + 0.00193185265241 * phi_sq) / sqrt(1.0 - 0.00669437999013 * phi_sq)));
}

/// calculates distance from the water surface where pressure is p0 to the point, where pressure is p
float UCNL_WPHX_depth_by_pressure_calc(float p, float p0, float rho, float g)
{
  return 100.0 * (p - p0) / (rho * g);
}

// Calculates pressure of a water column with given height (distance between
// the water surface and the given point) assuming constant water density
// h - depth, m
// p0 - atmospheric pressure, mBar
// rho - water density, kg/m^3
// g - gravity acceleration, m/s^2
float UCNL_WPHX_pressure_by_depth_calc(float h, float p0, float rho, float g)
{
  return h * rho * g / 100.0 + p0;
}

// Calculated the freezing temperature of seawater (in °C) with specified pressure and salinity.
// According to:
// Algorithms for computation of fundamental properties of seawater.
// Unesco technical papers in marine science vol. 44, 1983, pp. 30
// https://darchive.mblwhoilibrary.org/bitstream/handle/1912/2470/059832eb.pdf
// p - pressure, mBar
// s - PSU
float UCNL_WPHX_water_fpoint_calc(float p, float s)
{
  return (-0.0575 + 1.710523E-3 * sqrt(s) - 2.154996E-4 * s) * s - 7.53E-6 * p;
}
