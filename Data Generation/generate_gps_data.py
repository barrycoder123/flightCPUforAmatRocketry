#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Created on Thu Jan 26 11:13:09 2023

@author: zrummler
'''

import sys
import csv # for writing to CSV
import numpy as np
import pandas as pd # for reading CSV
import matplotlib.pyplot as plt


sys.path.append('../Flight Algorithms')

import earth_model as em

dt = 1 # GPS data arrives every second
GPS_POS_NOISE = 2.5
GPS_VEL_NOISE = 0.05
GPS_ACCEL_NOISE = 0.05

def add_noise(arr, noise):
    
    noise = np.random.uniform(low=-noise, high=noise, size=len(arr))
    arr += noise

if __name__ == "__main__":

    # open file and extract necessary data
    file_data = pd.read_csv("traj_raster_30mins_20221115_160156.csv")
    t_sec = np.array(file_data["t [sec]"])
    dt = np.average(np.diff(t_sec))
    print(dt)
    
    real_X = np.array(file_data["r_ecef_X"])
    real_Y = np.array(file_data["r_ecef_Y"])
    real_Z = np.array(file_data["r_ecef_Z"])
    
    # add noise to GPS data before conversion (since variance is in meters, not degrees)
    add_noise(real_X, GPS_POS_NOISE)
    add_noise(real_Y, GPS_POS_NOISE)
    add_noise(real_Z, GPS_POS_NOISE)
    
    # generate GPS position
    [lat_p, long_p, h_p] = em.ecef2lla(np.array([real_X, real_Y, real_Z]))
    
    # delete entries that aren't multiples of 10

    lat_p[~(np.arange(len(lat_p)) % 10 == 0)] = np.nan
    long_p[~(np.arange(len(long_p)) % 10 == 0)] = np.nan
    h_p[~(np.arange(len(h_p)) % 10 == 0)] = np.nan

    
    # # generate GPS velocity
    # [lat_v, long_v, h_v] = generateGPSDiff(lat_p, long_p, h_p)
    
    # # generate GPS accel
    # [lat_a, long_a, h_a] = generateGPSDiff(lat_v, long_v, h_v)
    
    # generate barometric pressure
    pressure = em.alt2pres(h_p)
    
    
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
    plt.plot(lat_p)
    plt.title("GPS Altitude (m)")
    plt.xlabel("Samples")
    plt.ylabel("Meters")
    
    
    plt.figure()
    [x, y, z] = em.lla2ecef([lat_p, long_p, h_p])
    plt.plot(x)
    plt.plot(y)
    plt.plot(z)
    plt.title("X Altitude (m)")
    plt.xlabel("Samples")
    plt.ylabel("Meters")
    
    # add noise to barometer data ?
    #add_noise(pressure)
    
    with open("gps_and_barometer_data.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
    
        # Write headers
        writer.writerow(["t [sec]", "GPS_lat", "GPS_long", "GPS_alt", "Barometer_Pa"])
    
        # Write data
        for i in range(len(lat_p)):
            writer.writerow([t_sec[i], lat_p[i], long_p[i], h_p[i], pressure[i]])



