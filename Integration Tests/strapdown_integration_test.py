#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 17:11:00 2023

@author: zrummler
"""

import sys
import time
import numpy as np

sys.path.append('../Flight Algorithms')
sys.path.append('../Data Collection')
sys.path.append('../Data Logging')
sys.path.append('../Test Data')

import strapdown as sd
import data_logger as dl
import data_collection_wrapper as dc

# main
if __name__ == "__main__":
    # ========================== setup ==========================
    print("IMU Strapdown Simulation")
    
    # Initialize the Data Collector module
    collector = dc.DataCollector().create()
    num_points = collector.num_points

    # initialize State
    kalman_state, q_true = collector.get_initial_state_and_quaternion()
    x = np.concatenate((kalman_state[0:6], q_true))
    
    # Initialize the Data Logging module
    logger = dl.DataLogger(kalman_state, q_true, num_points)

    # ========================== strapdown ==========================
    print("Running IMU strapdown...")
    start_time = time.time()
    collector.start_timer()
    logger.start_timer()
    for i in range(num_points):

        # get the next IMU reading
        accel, gyro, dt = collector.get_next_imu_reading()

        # grab our current PVA estimate
        r_ecef, v_ecef, q_e2b = x[0:3], x[3:6], x[6:10]

        # Run an iteration of the strapdown
        r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, accel, gyro, dt)

        # Write values back to estimation matrix
        x = np.concatenate((r_ecef_new, v_ecef_new, q_e2b_new))
        
        # Log the values for later viewing
        logger.save_state_to_buffer(x, q_e2b_new, np.concatenate((accel, gyro)), None)

    # ========================== plotting ==========================
    filename = logger.write_buffer_to_file()
    
    #dl.plot_file_contents(filename)
    logger.print_position_drift()
    
    end_time = time.time()
    print("EXECUTION TIME: ", end_time - start_time)

