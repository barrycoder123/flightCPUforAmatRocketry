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
import data_collection as dc

if __name__ == "__main__":
    
    # initialize state vectors and matrices
    dc.reset()
    x = kf.initialize_ekf_state_vector()
    q_true = kf.initialize_global_quaternion()
    P, Q, R, F = kf.initialize_ekf_matrices(x, q_true)
    
    # Initialize EKF
    ekf = kf.EKF(x, q_true, P, Q, R, kf.f, F, kf.h, kf.H)

    # EKF loop
    print("Running Kalman Filtering Simulation")
    while 1:
        if dc.done():
            break
        
        # Predict state
        ekf.predict()
        
        # If new GPS reading, update state
        if dc.gps_is_ready():
            lla, dt = dc.get_next_gps_reading()
            baro = dc.get_next_barometer_reading() # TODO: implement
            
            z = np.concatenate((lla, baro))
            ekf.update(z) 
        
    print("Finished Loop")
            