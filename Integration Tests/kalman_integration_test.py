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
import data_logging_wrapper as dl
import data_collection_wrapper as dc

importlib.reload(kf)
importlib.reload(dc)
importlib.reload(dl)

if __name__ == "__main__":
    """
    1. Collects input data and runs Kalman Filtering
    2. Logs the results (plotting)
    """
    
    # ========================== setup ==========================
    print("Kalman Filtering Simulation")

    # Initialize the Data Collector module
    collector = dc.DataCollector().create()
    num_points = collector.num_points
    
    # Initialize the Extended Kalman Filter module
    x, q_true = collector.get_initial_state_and_quaternion()
    ekf = kf.EKF(x, q_true)
    
    # Initialize the Data Logging module
    logger = dl.DataLogger(num_points).create()

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

    # ========================== plotting ==========================
    print("Logging results...")

    logger.show()