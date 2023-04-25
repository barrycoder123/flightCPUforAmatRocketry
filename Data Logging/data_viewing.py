#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 13:48:37 2023

@author: zrummler
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('../Flight Algorithms')

import earth_model as em

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
    #print(PVA_est.shape)
    
    # cuts out the really bad data ...
    if filename == "fells_loop.csv": #2100
        PVA_est = np.concatenate((PVA_est[:, 660:2000], PVA_est[:, 2315:]), axis=1)
       
    if filename == "capen_street.csv":
        PVA_est = np.concatenate((PVA_est[:,1400:2020], PVA_est[:,2033:]), axis=1)
    
    return PVA_est

    # BAD SECTIONS:
        
    # [450, 660]     [2100,2315]
        
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
    
    #BBox = ((df.longitude.min(),   df.longitude.max(), df.latitude.min(), df.latitude.max())
            
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
    ax.set_title('Plotting GPS Data on Map')
    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])
    ax.imshow(image, zorder=0, extent = BBox, aspect= 'equal')

    #plot_file_contents(file)

    
if __name__ == "__main__":
    
    day_1 = ["track_straight", "track_loop", "walk_to_track"]
    day_2 = ["first_drive", "capen_street", "fells_loop"]
    
    for log in day_2:
        plot_results_on_map(log)
    
    