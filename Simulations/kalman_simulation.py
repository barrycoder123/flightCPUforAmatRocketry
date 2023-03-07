#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 21 12:46:07 2023

@author: zrummler

PURPOSE: Test code for Kalman Filter simulation

"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append('../Flight Algorithms')
sys.path.append('../Data Generation')

import kalman as kf
import earth_model as em
import data_collection as dc

if __name__ == "__main__":
    
    # Import and format data
    print("Kalman Filtering Simulation")
    print("Opening file for truth data...")
    data = pd.read_csv("../Data Generation/traj_raster_30mins_20221115_160156.csv").to_numpy();    
    PVA_truth = data[:, 1:11];
    
    # Holds our results, for plotting
    print("Initializing arrays...")
    PVA_est = np.zeros((dc.num_points, 10))
    
    # Initialize state vectors and matrices
    dc.reset()
    x = kf.initialize_ekf_state_vector()
    q_true = kf.initialize_global_quaternion()
    P, Q, R, F = kf.initialize_ekf_matrices(x, q_true)
    ekf = kf.EKF(x, q_true, P, Q, R, kf.f, F, kf.h, kf.H)

    print("Running Extended Kalman Filter...")
    i = 0
    while 1:
        if dc.done():
            #print(i)
            break
        
        # Prediction
        ekf.predict()
        
        if dc.gps_is_ready():
            # read GPS and barometer
            lla, dt = dc.get_next_gps_reading()
            baro = dc.get_next_barometer_reading() # TODO: implement
            
            # Update state
            z = np.concatenate((lla, baro))
            ekf.update(z) 
        
        # FOR SIMULATION ONLY: write to results vector
        PVA_est[i, 0:3] = ekf.x[0:3]
        i += 1
        
        
    print("Plotting results...")
    ## PLOT POSITION
    plt.figure()
    plt.plot(PVA_truth[:, 0])
    plt.plot(PVA_est[:, 0])
    plt.title("X POSITION, WITH KALMAN FILTERING (GPS + IMU)")
    plt.legend(["Truth","Estimated"])
    plt.figure()
    plt.plot(PVA_truth[:, 1])
    plt.plot(PVA_est[:, 1])
    plt.title("Y POSITION, WITH KALMAN FILTERING (GPS + IMU)")
    plt.legend(["Truth","Estimated"])
    plt.figure()
    plt.plot(PVA_truth[:, 2])
    plt.plot(PVA_est[:, 2])
    plt.title("Z POSITION, WITH KALMAN FILTERING (GPS + IMU)")
    plt.legend(["Truth","Estimated"])
           