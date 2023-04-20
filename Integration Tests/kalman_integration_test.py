#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 16:06:53 2023

@author: zrummler

PURPOSE: used to test the Kalman Filter on the actual hardware

"""

import importlib

import sys
import numpy as np

sys.path.append('../Flight Algorithms')
sys.path.append('../Data Collection')
sys.path.append('../Data Logging')
sys.path.append('../Test Data')

import time
import kalman as kf
import data_logger as dl
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
    logger = dl.DataLogger(x, q_true, num_points)

    # ========================== filter ==========================
    print("Running Extended Kalman Filter...")
    start_time = time.time()
    collector.start_timer()
    logger.start_timer()
    for i in range(num_points):
        
        # Read IMU
        accel, gyro, dt = collector.get_next_imu_reading()
        
        # Prediction
        ekf.predict(accel, gyro, dt)

        # Read GPS and barometer -- these return None if no new data
        # baro = collector.get_next_barometer_reading()
        lla = collector.get_next_gps_reading()

        # Update
        baro = None
        ekf.update(lla, baro, sigma_gps=2, sigma_baro=10) # try variance = 10
        
        # Log the data
        logger.save_state_to_buffer(ekf.x, ekf.q_e2b, np.concatenate((accel,gyro)), lla)

    # ========================== plotting ==========================
    print("Logging results...")
    #filename = logger.write_buffer_to_file()
    
    #dl.plot_file_contents(filename)
    logger.print_position_drift()
    
    end_time = time.time()
    print("EXECUTION TIME: ", end_time - start_time)
