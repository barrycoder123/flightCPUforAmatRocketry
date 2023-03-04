#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler
"""
import sys
import pandas as pd
import numpy as np

sys.path.append('../Data Generation')

# code for simulation on PC

imu_reading_number = 0
gps_reading_number = 0
last_time = 0

GPS_UPDATE_TIME = 1

# reading IMU data
file_data = pd.read_csv("../Data Generation/traj_raster_30mins_20221115_160156.csv")
imu_t_sec = np.array(file_data["t [sec]"])
accel_X = np.array(file_data["accel_X [m/s^2]"])
accel_Y = np.array(file_data["accel_Y [m/s^2]"])
accel_Z = np.array(file_data["accel_Z [m/s^2]"])
gyro_X = np.array(file_data["gyro_X [rad/s]"])
gyro_Y = np.array(file_data["gyro_X [rad/s]"])
gyro_Z = np.array(file_data["gyro_X [rad/s]"])
    
# reading GPS data
file_data = pd.read_csv("../Data Generation/gps_and_barometer.csv")
gps_t_sec = np.array(file_data["t [sec]"])
gps_lat = np.array(file_data["GPS_lat"])
gps_long = np.array(file_data["GPS_long"])
gps_alt = np.array(file_data["GPS_alt"])


""" returns the next IMU reading (simulation) """
def get_next_imu_reading():
    global imu_reading_number

    # extract the next acceleration and angular rotation
    accel_xyz = np.array([accel_X[imu_reading_number], accel_Y[imu_reading_number], accel_Z[imu_reading_number]])
    gyro_xyz = np.array([gyro_X[imu_reading_number], gyro_Y[imu_reading_number], gyro_Z[imu_reading_number]])
    
    # time step computation (dt)
    if imu_reading_number == 0:
        dt = imu_t_sec[1] - imu_t_sec[0]
    else:
        dt = imu_t_sec[imu_reading_number] - imu_t_sec[imu_reading_number-1]
        
    imu_reading_number += 1    
    
    return accel_xyz, gyro_xyz, dt
    

""" returns the next GPS reading (simulation) """
def ping_gps_for_reading():
    global gps_reading_number
    
    # extract the next lat, long, altitude
    lat = gps_lat[gps_reading_number]
    long = gps_long[gps_reading_number]
    alt = gps_alt[gps_reading_number]
    
    # time step computation (dt)
    if gps_reading_number == 0:
        dt = gps_t_sec[10] - gps_t_sec[0]
    else:
        dt = gps_t_sec[imu_reading_number] - gps_t_sec[imu_reading_number-10]
   
    gps_reading_number += 10    
   
    return lat, long, alt, dt
    
""" how to check if the GPS is ready for next data """
def gps_is_ready(running_time):
    global last_time
    
    if (running_time - last_time) >= GPS_UPDATE_TIME:
        last_time = running_time
        return True
    
    last_time = running_time
    return False
    
    