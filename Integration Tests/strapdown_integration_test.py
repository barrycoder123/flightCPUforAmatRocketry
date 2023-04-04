#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 17:11:00 2023

@author: zrummler
"""

import sys
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

    # Initialize the Data Collector module
    collector = dc.DataCollector().create()
    num_points = collector.num_points
    
    # initialize State
    kalman_state, q_true = collector.get_initial_state_and_quaternion()
    x = np.concatenate((kalman_state[0:6], q_true))
    
    # Initialize the Data Logging module
    colnames = ["t", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "q_scalar", "q_i", "q_j", "q_k"]
    logger = dl.DataLogger(kalman_state, q_true, colnames, num_points)

    # Run the strapdown for all data
    print("Running strapdown simulation")
    logger.start_timer()
    for i in range(num_points):

        # get the next IMU reading
        accel, gyro, dt = collector.get_next_imu_reading()
        dV_b_imu = accel * dt
        dTh_b_imu = gyro * dt

        # grab our current PVA estimate
        r_ecef, v_ecef, q_e2b = x[0:3], x[3:6], x[6:10]

        # Run an iteration of the strapdown
        r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt)

        # Write values back to estimation matrix
        x = np.concatenate((r_ecef_new, v_ecef_new, q_e2b_new))
        
        # Log the values for later viewing
        logger.save_state_to_buffer(x, q_e2b_new)

    logger.write_buffer_to_file()
    print("Plotting results...")
    
    logger.plot_file_contents()
    logger.print_file_contents()

