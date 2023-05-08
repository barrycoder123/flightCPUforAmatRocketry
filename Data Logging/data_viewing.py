#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 13:48:37 2023

@author: zrummler

This file lets you view your .csv data overlayed on a map.

How to run:
    - Assume you have a .csv file called "test_name.csv" ... call the function 
    plot_results_on_map("test_name"). The function prints min and max latitude 
    and longitude values. It will fail because you don't have a map image to 
    plot it against.
    - To create the map image, head to openstreetmap.com. Create a map with 
    these exact min and max lat/long values. Export it as a PNG image called
    "test_name.png" (the exact same name as your .csv file)
    - Now you should be able to run the same code again and it will plot the
    results on the map you just created online.
    - Enjoy!!
"""

import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../Flight Algorithms')

import earth_model as em
import data_logger as dl


"""
The following files are separate from the DataLogger class but are helpful for 
viewing the data after it has been written to a file
"""

def read_file_to_buffer(filename):
    """
    Reads the file into a numpy array
    
    Parameters:
        filename: (str) name of file, returned by DataLogger.write_buffer_to_file()
    """
    
    PVA_est = np.loadtxt(filename, delimiter=',', dtype=str).T
    PVA_est = PVA_est.astype(float)
    
    return PVA_est

        
def plot_file_contents(filename):
    """
    Plots the position, velocity, and attitude contents from the file
    
    Parameters:
        filename: (str) name of file, returned by DataLogger.write_buffer_to_file()
    """
    
    PVA_est = read_file_to_buffer(filename)
    
    tplot = np.arange(PVA_est.shape[1])
    _plotData(PVA_est[7:11, ::10], tplot[::10], axes=['A', 'B', 'C', 'D'], name='Quaternion', unit=None)
    _plotData(PVA_est[4:7, :], tplot, name='Velocity', subx0=True)
    _plotData(PVA_est[1:4, :], tplot, subx0=True)
    
    plt.show()  # needed to display the figures
 
    
def print_file_contents(filename):
    """
    Prints the entire scontents from the file
    
    Parameters:
        filename: (str) name of file, returned by DataLogger.write_buffer_to_file()
    """
    
    PVA_est = read_file_to_buffer(filename)
    num_points = PVA_est.shape[1]
    
    print("POSITION:")
    
    for i in range(num_points):
        print(PVA_est[0, i], PVA_est[1:4, i])
        
        
def _plotData(x, t, cov=None, name='Position', unit='m', subx0=False, decim_fact=1, tEnd=None, axes=None):
    """
    Private helper function which plots everything very nicely
    Credit: Tyler Klein

    Parameters
    ----------
    x : (M,N) ndarray
        matrix of measured values, where ``x[:,i]`` is the vector of measured quantities at time `i`
    x_true : (M,N) ndarray
        matrix of truth values, where ``x_true[:,i]`` is the vector of true values at time `i`. Same shape as ``x``
    t : (N,) ndarray
        time vector [seconds]
    cov : (M, M, N) ndarray
        if provided, specifies the covariance for the data. This will be plotted in the error graphs
    name : str, default='Position'
        plot name to appear in titles and axis labels
    unit : str, default='m'
        unit string for axis labels
    subx0 : bool, default=False
        if True, the true value at the first time will be subtracted from both vectors. This is useful when plotting quantities with a large offset,
        such as ECEF position
    decim_fact : int, default=1
        decimation factor for plotting
    tEnd : float
        if provided, the xlim will be set to end at this time
    axes : list
        axis names. Default is ['x', 'y', 'z']. Length match x.shape[0]
    """
    
    
    if axes is None:
        axes = ['X', 'Y', 'Z']

    if len(axes) != x.shape[0]:
        raise ValueError('len(axes) != x.shape[0]')

    x0_use = x[:, 0] if subx0 else np.zeros((len(axes),))

    fig, axs = plt.subplots(len(axes), 1, sharex=True, figsize=(13, 8))
    plt.suptitle(name, fontsize='xx-large', fontweight='black')

    tplot = t[::decim_fact]
    for i, row in enumerate(axs):
        data = x[i, ::decim_fact]
        row.plot(tplot, data - x0_use[i])
        row.set_xlabel('Time [sec]')
        row.set_ylabel(f'[{unit}]')
        row.set_title(f"{axes[i]}")
        row.grid(True)


        if tEnd is not None:
            row.set_xlim([0, tEnd])

    plt.tight_layout()
    return fig, axs


def plot_results_on_map(test_name):
    
    
    data_file = test_name + ".csv"
    image_file = test_name + ".png"
    #image_file = "fells_loop.png"
            
    PVA_est = read_file_to_buffer(data_file)
    position_est = PVA_est[1:4,:]
    lla_est = em.ecef2lla(position_est)
    
    latitude = lla_est[0]
    longitude = lla_est[1]
    
    min_lon = min(longitude)
    max_lon = max(longitude)
    min_lat = min(latitude)
    max_lat = max(latitude)
    
    BBox = ((min_lon, max_lon, min_lat, max_lat))
    print(BBox)
    image = plt.imread(image_file)
    
    fig, ax = plt.subplots(figsize = (8,7))
    ax.scatter(longitude, latitude, zorder=1, alpha= 0.2, c='b', s=10)
    ax.set_title('Recording ' + test_name)
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])
    ax.imshow(image, zorder=0, extent = BBox, aspect= 'equal')
    
    
def determine_track_distances():
    
    # ==================== get data from files ====================
    data_file = "track_straight.csv"
    image_file = "track_straight.png"
    
    # unpack data
    PVA_est = read_file_to_buffer(data_file)
    min_time = PVA_est[0,0]
    max_time = PVA_est[0,-1]
    num_points = PVA_est.shape[1]
    ten = 10 # r/programminghorror would be proud
    time_list = list(range(int(min_time),int(max_time),ten))

    # ==================== get the real data we need ====================
    r_ecef_ten_meters = np.zeros((3, ten))
    r_ecef_ten_meters[:,0] = PVA_est[1:4,0]
    
    t_idx = 1
    for i in range(num_points):
        col = PVA_est[:,i]
        
        # find the point at which we stopped on the track
        if (col[0] > time_list[t_idx]) and (col[0] < time_list[t_idx] + 1):
            if col[17] is not np.nan: # find the measurement corresponding to a GPS update
            
                print("Found col",col[0])
                real_col = PVA_est[:,i+10] #the next col has the best update
                r_ecef_ten_meters[:,t_idx] = real_col[1:4]
                t_idx = t_idx + 1
        
        if t_idx >= ten: # only need 10 points
            break
        
    # ==================== plot on a map ====================
    lla_est = em.ecef2lla(r_ecef_ten_meters)
    latitude = lla_est[0]
    longitude = lla_est[1]
    
    min_lon = min(longitude) - 5e-5
    max_lon = max(longitude) + 5e-5
    min_lat = min(latitude) - 5e-5
    max_lat = max(latitude) + 5e-5
    
    BBox = ((min_lon, max_lon, min_lat, max_lat))
    print(BBox)
    image = plt.imread(image_file)
    
    fig, ax = plt.subplots(figsize = (8,7))
    ax.scatter(longitude, latitude, zorder=1, alpha= 0.8, c='r', s=25)
    ax.set_title('Recording at 8.5 meter spacings on a running track')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])
    for i in range(10):
        txt = "Point " + str(i+1)
        ax.annotate(txt, (longitude[i], latitude[i]))
    ax.imshow(image, zorder=0, extent = BBox, aspect= 'equal')
    
    # ==================== determine distance (meters) between each data ====================
    actual_distance = 8.5
    prev_pos = r_ecef_ten_meters[:,0]
    error_diff_accum = 0
    for i in range(ten):
        pos = r_ecef_ten_meters[:,i]
        diff_xyz = pos - prev_pos
        diff_abs = round(np.linalg.norm(diff_xyz),2)
        error_diff = round(abs(diff_abs - actual_distance * i),2)
        print(round(diff_abs,2), actual_distance * i, error_diff)
        error_diff_accum += error_diff
    print("Average error:",error_diff_accum/10)
    
    
if __name__ == "__main__":
    
    day_1 = ["track_straight", "track_loop", "walk_to_track"]
    day_2 = ["first_drive", "fells_loop_1", "fells_loop_2"]
    
    # plot each test from a particular day
    plot_results_on_map(day_2[0])
    