#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 17:54:05 2023

@author: zrummler
"""

import numpy as np

WE = 7.2921151467e-05;
omega = np.array([0, 0, WE]);

''' generate GPS Position '''
# generate GPS [lat, long, h] from ECEF (x, y, z)
# https://stackoverflow.com/questions/56945401/converting-xyz-coordinates-to-longitutde-latitude-in-python
def ecef2lla(x, y, z):

    a = 6378137.0 #in meters
    b = 6356752.314245 #in meters

    f = (a - b) / a
    f_inv = 1.0 / f

    e_sq = f * (2 - f)                       
    eps = e_sq / (1.0 - e_sq)

    p = np.sqrt(np.multiply(x,x) + np.multiply(y,y))
    q = np.arctan2((z * a), (p * b))

    sin_q = np.sin(q)
    cos_q = np.cos(q)

    sin_q_3 = sin_q * sin_q * sin_q
    cos_q_3 = cos_q * cos_q * cos_q

    phi = np.arctan2((z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3))
    lam = np.arctan2(y, x)

    v = a / np.sqrt(1.0 - e_sq * np.sin(phi) * np.sin(phi))
    h   = (p / np.cos(phi)) - v

    lat = np.degrees(phi)
    lon = np.degrees(lam)
    
    return lat, lon, h


''' generate GPS Diff '''
# determine pressure from altitude
# https://pvlib-python.readthedocs.io/en/v0.2.2/_modules/pvlib/atmosphere.html
def alt2pres(altitude):
    '''
    Determine site pressure from altitude.

    Parameters
    ----------
    Altitude : scalar or Series
        Altitude in meters above sea level 

    Returns
    -------
    Pressure : scalar or Series
        Atmospheric pressure (Pascals)

    Notes
    ------
    The following assumptions are made

    ============================   ================
    Parameter                      Value
    ============================   ================
    Base pressure                  101325 Pa
    Temperature at zero altitude   288.15 K
    Gravitational acceleration     9.80665 m/s^2
    Lapse rate                     -6.5E-3 K/m
    Gas constant for air           287.053 J/(kgK)
    Relative Humidity              0%
    ============================   ================

    References
    -----------

    "A Quick Derivation relating altitude to air pressure" from Portland
    State Aerospace Society, Version 1.03, 12/22/2004.
    '''

    press = 100 * ((44331.514 - altitude) / 11880.516) ** (1 / 0.1902632)
    
    return press


# xyz2grav
#
# Ellipsoid Earth gravity model
def xyz2grav(x, y, z):
    j2 = 0.00108263
    mu = 3.986004418e14
    R = 6378137
    r = np.sqrt(x**2 + y**2 + z**2)
    sub1 = 1.5*j2*((R/r)**2)
    sub2 = (5*z**2)/(r**2)
    sub3 = (-mu)/(r**3)
    sub4 = sub3 * (1 - sub1 * (sub2 - 1));
    gx = x * (sub4)
    gy = y * (sub4);
    gz = z * (sub3) * (1 - sub1 * (sub2 - 3));
    g = np.array([gx, gy, gz]);
    g = g[:] #Force column
    return g
