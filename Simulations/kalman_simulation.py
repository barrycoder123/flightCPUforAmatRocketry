#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 21 12:46:07 2023

@author: zrummler

PURPOSE: Runs a Kalman Filtering simulation with Draper's test data
Good for quick debugging/verification of the Kalman Filtering implementation

"""
import importlib

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append('../Flight Algorithms')
sys.path.append('../Data Collection')
sys.path.append('../Test Data')

import kalman as kf
import earth_model as em
import data_collection_wrapper as dcw
from misc import plotDataAndError

importlib.reload(kf)
importlib.reload(em)
importlib.reload(dcw)

if __name__ == "__main__":
    '''
    1. Imports test data
    2. Runs Kalman Filtering on the data
    3. Compares results of Kalman Filter to truth data (run the code and see plots)
    '''
    print("Kalman Filtering Simulation")

    # Initialize the Data Collection module
    dc = dcw.DataCollector().create()
    num_points = dc.num_points
    
    # Initialize the EKF module
    x, q_true = dc.get_initial_state_and_quaternion()
    ekf = kf.EKF(x, q_true)
    
    # Initialize state vectors and matrices
    print("Initializing arrays...")
    PVA_est = np.zeros((10, num_points))
    PHist = np.zeros((9, 9, num_points))  # to store the covariance at each step

    # Import and format data
    print("Opening file for truth data...")
    data = pd.read_csv("../Test Data/traj_raster_30mins_20221115_160156.csv").to_numpy()
    PVA_truth = data[:, 1:11].T

    # ========================== filter ==========================
    print("Running Extended Kalman Filter...")

    for i in range(num_points):

        # Read IMU
        accel, gyro, dt = dc.get_next_imu_reading()
        z_imu = np.concatenate((accel, gyro))
        
        # Prediction
        ekf.predict(z_imu, dt)

        # Read GPS and barometer -- these return None if no new data
        baro = dc.get_next_barometer_reading()
        #lla = dc.get_next_gps_reading()
        lla = None

        # Update
        #baro = None
        #baro = baro#[:1] # try just one barometer before all three
        ekf.update(lla, baro, sigma_gps=5, sigma_baro=5) # try variance = 10

        # save the data
        PVA_est[:6, i] = ekf.x[:6]  # store ECEF position and velocity
        PVA_est[6:10, i] = ekf.q_e2b
        PHist[:, :, i] = ekf.P  # store the covariance

    # ========================== plotting ==========================
    print("Plotting results...")

    tplot = np.arange(PVA_est.shape[1])
    plotDataAndError(PVA_est[:3, :], PVA_truth[:3, :], tplot, subx0=True)
    plotDataAndError(PVA_est[3:6, :], PVA_truth[3:6, :], tplot, name='Velocity', subx0=True)
    #plotDataAndError(PVA_est[:3, ::10], PVA_truth[:3, ::10], tplot[::10], subx0=True)
    plotDataAndError(PVA_est[6:10, ::10], PVA_truth[6:10, ::10], tplot[::10], axes=['A', 'B', 'C', 'D'], name='Quaternion', unit=None)

    plt.show()  # needed to display the figures
