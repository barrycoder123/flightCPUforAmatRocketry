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
    
    # Holds our results, for plotting
    r_gps = np.zeros((dc.num_points, 3))
    r_filtered = np.zeros((dc.num_points, 3))
    
    # initialize state vectors and matrices
    dc.reset()
    x = kf.initialize_ekf_state_vector()
    q_true = kf.initialize_global_quaternion()
    P, Q, R, F = kf.initialize_ekf_matrices(x, q_true)
    
    # Initialize EKF
    ekf = kf.EKF(x, q_true, P, Q, R, kf.f, F, kf.h, kf.H)

    # EKF loop
    print("Running Kalman Filtering Simulation")
    i = 0
    while 1:
        if dc.done():
            #print(i)
            break
        
        # Predict state
        ekf.predict()
        
        
        if dc.gps_is_ready():
            
            # read GPS and barometer
            lla, dt = dc.get_next_gps_reading()
            baro = dc.get_next_barometer_reading() # TODO: implement
            
            # Update state
            z = np.concatenate((lla, baro))
            ekf.update(z) 
        
            
        # update results vectors
        #print(lla)
        xyz = em.lla2ecef(lla)
        r_gps[i, :] = xyz
        r_filtered[i, :] = ekf.x[0:3]
        i += 1
        
        
    print("Finished Loop")
    
    ## PLOT POSITION
    plt.figure()
    plt.plot(r_gps[:, 0])
    plt.plot(r_filtered[:, 0])
    plt.title("X POSITION")
    plt.legend(["GPS","Filtered"])
    plt.figure()
    plt.plot(r_gps[:, 1])
    plt.plot(r_filtered[:, 1])
    plt.title("Y POSITION")
    plt.legend(["GPS","Filtered"])
    plt.figure()
    plt.plot(r_gps[:, 2])
    plt.plot(r_filtered[:, 2])
    plt.title("Z POSITION")
    plt.legend(["GPS","Filtered"])
           