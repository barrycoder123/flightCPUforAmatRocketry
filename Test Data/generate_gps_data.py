#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 26 11:13:09 2023

@author: zrummler
"""

import sys
import os
import numpy as np
import pandas as pd  # for reading CSV
import matplotlib.pyplot as plt
import argparse

sys.path.append(os.path.join('..', 'Flight Algorithms'))
sys.path.append(os.path.join('..', 'Simulations'))

import earth_model as em
from misc import get_ecef_column_names  # for later user

dt = 1  # GPS data arrives every second

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--filename', help='trajectory .csv file', default='traj_raster_30mins_20221115_160156.csv')
    parser.add_argument('-g', '--gps_sigma', help='position error standard deviation [m]', default=5, type=float)
    parser.add_argument('-b', '--baro_sigma', help='altitude error standard deviation [m]', default=10, type=float)
    args = parser.parse_args()

    pos_cols, vel_cols, quat_cols = get_ecef_column_names()  # for later user

    # open file and extract necessary data
    df = pd.read_csv(args.filename)

    # extract ECEF position and convert to LLA
    r_ecef = df[pos_cols].to_numpy().T

    # NOTE: the noise generation should really go after the conversion, but it's easier like this even tho it's technically not correct
    r_ecef_noisy = r_ecef + args.gps_sigma * np.random.randn(*r_ecef.shape)

    # generate GPS position
    r_lla = em.ecef2lla(r_ecef_noisy)  # [deg, deg, m]

    # delete entries that aren't multiples of 10
    # todo: this should be a parameter input
    r_lla_modified = np.nan + np.ones(r_lla.shape)
    r_lla_modified[:, ::10] = r_lla[:, ::10]

    # generate barometric altitude in meters
    baro_alt = em.ecef2lla(r_ecef)[2, :].reshape(-1,1) # em.alt2pres(r_lla[2, :]).reshape(-1, 1)
    baro1_alt = baro_alt + args.baro_sigma * np.random.randn(*baro_alt.shape)
    baro2_alt = baro_alt + args.baro_sigma * np.random.randn(*baro_alt.shape)
    baro3_alt = baro_alt + args.baro_sigma * np.random.randn(*baro_alt.shape)
    plt.plot(baro1_alt)

    colnames = ["GPS_lat (deg)", "GPS_long (deg)", "GPS_alt (m)", "Baro1_alt (m)", "Baro2_alt (m)", "Baro3_alt (m)"]
    df_new = pd.DataFrame(np.hstack((r_lla_modified.T, baro1_alt, baro2_alt, baro3_alt)), columns=colnames)
    df_new.insert(0, 't [sec]', df["t [sec]"])
    df_new.to_csv("gps_and_barometer_data.csv", index=False)

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
    plt.plot(r_lla[2,:])
    plt.title("GPS Altitude (m)")
    plt.xlabel("Samples")
    plt.ylabel("Meters")

    plt.figure()
    [x, y, z] = em.lla2ecef(r_lla)
    plt.plot(x)
    plt.plot(y)
    plt.plot(z)
    plt.title("X Altitude (m)")
    plt.xlabel("Samples")
    plt.ylabel("Meters")
