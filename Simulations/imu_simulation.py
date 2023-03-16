#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 21 12:46:07 2023

@author: zrummler

PURPOSE: Test code for IMU simulation
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append('../Flight Algorithms')
sys.path.append('../Data Collection')
sys.path.append('../Test Data')

import strapdown as sd
import file_data_collection as fdc

from misc import get_imu_column_names, get_ecef_column_names, plotDataAndError#, #progressBar

# main
if __name__ == "__main__":

    # ====================== configuration ======================
    imu_file = r"../Test Data/traj_raster_30mins_20221115_160156.csv"
    # imu_file = r"C:\Users\tqk0611\Documents\Haystack\goonies_repo\analysis\traj\traj_raster_30mins_20230310_081326.csv"

    # ===========================================================
    t_col, accel_cols, gyro_cols, mag_cols = get_imu_column_names()  # for later user
    pos_cols, vel_cols, quat_cols = get_ecef_column_names()  # for later user

    dc = fdc.FileDataCollector()
    num_points = dc.num_points

    # Import and format data
    df = pd.read_csv(imu_file)  # DataFrame
    #numPoints = len(df)
    # data = pd.read_csv(imu_file).to_numpy().T
    PVA_truth = df[pos_cols + vel_cols + quat_cols].to_numpy().T
    dt = np.mean(np.diff(df[t_col]))  # [seconds]

    # Initialize arrays
    PVA_est = np.zeros(PVA_truth.shape)
    PVA_est[:, 0] = PVA_truth[:, 0]  # Store initial conditions in first col of estimate

    # initialize state
    x = PVA_truth[:10, 0].reshape(-1, 1)  # initial state

    # Run the strapdown for all data
    print(f"Running strapdown simulation with dt = {1e3 * dt:.0f} ms")
    for i in range(num_points - 1):

        # simulate getting the next IMU reading
        accel, gyro, dt = dc.get_next_imu_reading()
        dV_b_imu = accel * dt
        dTh_b_imu = gyro * dt

        # grab our current PVA estimate
        r_ecef = PVA_est[0:3, i]  # ECEF position [m]
        v_ecef = PVA_est[3:6, i]  # ECEF velocity [m/s]
        q_e2b = PVA_est[6:10, i]  # ECEF-to-body Quaternion

        # Run an iteration of the strapdown
        r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt)

        # Write values back to estimation matrix
        PVA_est[0:3, i + 1] = r_ecef_new
        PVA_est[3:6, i + 1] = v_ecef_new
        PVA_est[6:10, i + 1] = q_e2b_new

    print("Plotting results...")


    # =======================================================================================
    plotDataAndError(PVA_est[6:, :], PVA_truth[6:, :], df[t_col], unit=None, axes=['A', 'B', 'C', 'D'], name='Quaternion')
    plotDataAndError(PVA_est[3:6, :], PVA_truth[3:6, :], df[t_col], name='Velocity', unit='m/s')
    plotDataAndError(PVA_est[:3, :], PVA_truth[:3, :], df[t_col], subx0=True)

    plt.show()
