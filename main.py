#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 16:06:53 2023

@author: zrummler

Main function!
"""

import importlib

import sys
import numpy as np

sys.path.append('../Flight Algorithms')
sys.path.append('../Data Collection')
sys.path.append('../Data Logging')
sys.path.append('../Test Data')

import kalman as kf
import earth_model as em
import data_logging_wrapper as dl
import data_collection_wrapper as dc

importlib.reload(kf)
importlib.reload(em)
importlib.reload(dc)
importlib.reload(dl)

if __name__ == "__main__":
    """
    1. Imports test data
    2. Runs Kalman Filtering on the data
    3. Compares results of Kalman Filter to truth data (run the code and see plots)
    """
    print("Kalman Filtering Simulation")

    # Initialize the Data Collector module
    collector = dc.DataCollector().create()
    num_points = collector.num_points
    
    # Initialize the Extended Kalman Filter module
    x, q_true = collector.get_initial_state_and_quaternion()
    ekf = kf.EKF(x, q_true)
    
    # Initialize the Data Logging module
    logger = dl.DataLogger(num_points).create()
    
    # For debugging: store the covariance at each step
    PHist = np.zeros((9, 9, num_points))  # to store the covariance at each step

    # ========================== filter ==========================
    print("Running Extended Kalman Filter...")
    for i in range(num_points):

        # Read IMU
        accel, gyro, dt = collector.get_next_imu_reading()
        z_imu = np.concatenate((accel, gyro))
        
        # Prediction
        ekf.predict(z_imu, dt)

        # Read GPS and barometer -- these return None if no new data
        baro = collector.get_next_barometer_reading()
        lla = collector.get_next_gps_reading()

        # Update
        baro = None
        ekf.update(lla, baro, sigma_gps=5, sigma_baro=10) # try variance = 10
        
        # Log the data
        logger.save(ekf.x, ekf.q_e2b)

        # For debugging: store the covariance
        PHist[:, :, i] = ekf.P  # store the covariance

    # ========================== plotting ==========================
    print("Plotting results...")

    logger.show()
    #plotDataAndError(PVA_est[:3, ::10], PVA_truth[:3, ::10], tplot[::10], subx0=True)


    # fig, axs = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    # for i, (ax, lab) in enumerate(zip(axs, ['X', 'Y', 'Z'])):
    #     ax.plot(PVA_truth[i, :] - PVA_truth[i, 0], label='Truth')
    #     ax.plot(PVA_est[i, :] - PVA_truth[i, 0], label='Estimate')
    #     ax.set_title(f"{lab} ECEF Position (minus start)\nWITH KALMAN FILTERING (GPS + IMU)")
    #     ax.legend()
    #     ax.grid(True)
    # plt.tight_layout()