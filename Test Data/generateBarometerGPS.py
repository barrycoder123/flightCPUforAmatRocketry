#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Created on Thu Jan 26 11:13:09 2023

@author: zrummler
'''

from pandas import *
import numpy as np
import matplotlib.pyplot as plt
import math
import csv

dt = 0.1
GPS_POS_NOISE = 2.5
GPS_VEL_NOISE = 0.05
GPS_ACCEL_NOISE = 0.05

def add_noise(arr, noise):
    
    noise = np.random.uniform(low=-noise, high=noise, size=len(arr))
    arr += noise

''' generate GPS Position '''
# generate GPS [lat, long, h] from ECEF (x, y, z)
# https://stackoverflow.com/questions/56945401/converting-xyz-coordinates-to-longitutde-latitude-in-python
def generateGPSPos(x, y, z):

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

''' generate GPS Diff'''
# computes GPS velocity by taking the derivative of GPS position

# velocity = (change-in-position) / (change-in-time)
# lat_v[i] = (lat_p[i] - lat_p[i-1]) / dt

def generateGPSDiff(lat_p, lon_p, h_p):
    
    lat_v = np.diff(lat_p) / dt 
    lon_v = np.diff(lon_p) / dt
    h_v = np.diff(h_p) / dt
    
    return lat_v, lon_v, h_v

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

if __name__ == "__main__":

    # open file and extract necessary data
    file_data = read_csv("traj_raster_30mins_20221115_160156.csv")
    t_sec = np.array(file_data["t [sec]"])
    real_X = np.array(file_data["r_ecef_X"])
    real_Y = np.array(file_data["r_ecef_Y"])
    real_Z = np.array(file_data["r_ecef_Z"])
    
    # add noise to GPS data before conversion (since variance is in meters, not degrees)
    #add_noise(real_X, GPS_POS_NOISE)
    #add_noise(real_Y, GPS_POS_NOISE)
    #add_noise(real_Z, GPS_POS_NOISE)
    
    # generate GPS position
    [lat_p, long_p, h_p] = generateGPSPos(real_X, real_Y, real_Z)
    
    # generate GPS velocity
    [lat_v, long_v, h_v] = generateGPSDiff(lat_p, long_p, h_p)
    
    # generate GPS accel
    [lat_a, long_a, h_a] = generateGPSDiff(lat_v, long_v, h_v)
    
    # generate barometric pressure
    pressure = alt2pres(h_p)
    
    
    # =========================================================================
    # # plot GPS for sanity check
    # BBox = ((min(long_p), max(long_p),      
    #          min(lat_p), max(lat_p)))
    # fig, ax = plt.subplots(figsize = (8,7))
    # ax.scatter(long_p, lat_p, zorder=1, alpha= 0.2, c='b', s=10)
    # ax.set_title('GPS Lat-Long (superimposed over city of Cambridge)')
    # ax.set_xlim(min(long_p),max(long_p))
    # ax.set_ylim(min(lat_p),max(lat_p))
    # map_image = plt.imread('map.png')
    # ax.imshow(map_image, zorder=0, extent=BBox, aspect= 'equal')
    # =========================================================================
    
    # plot altitude (above sea level) for sanity check
    plt.figure()
    plt.plot(h_p)
    plt.title("GPS Altitude (m)")
    plt.xlabel("Samples")
    plt.ylabel("Meters")
    
    
    # add noise to barometer data ?
    #add_noise(pressure)
    
    with open("gps_and_barometer_data.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
    
        # Write headers
        writer.writerow(["t [sec]", "X","Y","Z","GPS_lat", "GPS_long", "GPS_alt", "Barometer_Pa"])
    
        # Write data
        for i in range(len(lat_p)):
            writer.writerow([t_sec[i], real_X[i], real_Y[i], real_Z[i], lat_p[i], long_p[i], h_p[i], pressure[i]])



