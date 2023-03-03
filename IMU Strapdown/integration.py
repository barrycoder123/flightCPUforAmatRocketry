#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 21 12:46:07 2023
@author: zrummler
"""

import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from strapdown import *
from readsensors import *

# main
if __name__ == "__main__":
    
    timesteps = 200;
    offset = np.array([0.2687, 0.6816, 0.43375])
    
    # Initialize arrays
    PVA_est = np.zeros((timesteps, 10))
    x0 = [1526803, -4462969, 4278803, 0, 0, 0, 0.90179, -0.39979, -0.0665085, -0.1500215]
    x0 = x0[:]; # Force column vector
    PVA_est[0] = x0 # Store initial conditions in first col of estimate
    testing = np.zeros((timesteps, 3))
    # Run the strapdown for all data
    for i in range(timesteps - 1): 
    
        # Extract values from IMU
        last_gyro,last_accel,last_baro,dt,last_time = collectdata(last_gyro, last_accel, last_baro, last_time, gyro, accel, baro);
        #dt = 0.1
        
        #last_accel = np.array([0, 0, 9.82])
        #last_gyro = np.array([0, 0, 0])
        
        #if i < 10:
        # PVA_est[i, 6:10] = deltaAngleToDeltaQuat(last_gyro)
        # 1 0 0 0
        
        dV_b_imu = dt * (np.array(last_accel) + offset); # measured delta-V in the body frame [m/s]
        dTh_b_imu = dt * np.array(last_gyro); # measured delta-theta in the body frame [rad]
    
        r_ecef = PVA_est[i, 0:3]; # ECEF position [m]
        v_ecef = PVA_est[i, 3:6]; # ECEF velocity [m/s]
        q_e2b = PVA_est[i, 6:10]; # ECEF-to-body Quaternion
        
        print(v_ecef)
        #print(q_e2b);
        #print(dt)
        #testing[i] = np.array(last_accel) + offset
        #testing[i] = np.array(last_gyro)
        
        #print(imu.offsets_accelerometer)
        # Run an iteration of the strapdown
        r_ecef_new, v_ecef_new, q_e2b_new = strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);
    
        # Write values back to estimation matrix
        PVA_est[i + 1, 0:3] = r_ecef_new;
        PVA_est[i + 1, 3:6] = v_ecef_new;
        PVA_est[i + 1, 6:10] = q_e2b_new;
    
    print(np.mean(testing, axis=0))
    
    '''
    print("Plotting results...")
    
    """ this is a terrible way to do this """
    
    ## PLOT POSITION
    plt.figure()
   
    plt.plot(PVA_est[:, 0])
    plt.title("X POSITION")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 1])
    plt.title("Y POSITION")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 2])
    plt.title("Z POSITION")
    plt.legend(["Truth","Estimation"])
    
    
    ## PLOT VELOCITY
    plt.figure()
    plt.plot(PVA_est[:, 3])
    plt.title("X VELOCITY")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 4])
    plt.title("Y VELOCITY")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 5])
    plt.title("Z VELOCITY")
    plt.legend(["Truth","Estimation"])
    
    ## PLOT ATTITUDE
    plt.figure()
    plt.plot(PVA_est[:, 6])
    plt.title("SCALAR QUATERNION")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 7])
    plt.title("I QUATERNION")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 8])
    plt.title("J QUATERNION")
    plt.legend(["Truth","Estimation"])
    plt.figure()
    plt.plot(PVA_est[:, 9])
    plt.title("K QUATERNION")
    plt.legend(["Truth","Estimation"])
    '''