#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler


PURPOSE: Library for data collection from hardware sensors


FUNCTIONS:
    get_next_imu_reading()
    gps_is_ready() - bool
    get_next_gps_reading()
    get_next_barometer_reading
    get_first_quaternion()
    
"""

import pandas as pd
import numpy as np

import platform
platform.processor()


DEVICES = ["BEAGLEBONE", "TESTING ON PC"]
DEVICE = DEVICES[1]

""" code for running on the Raspberry Pi or Beaglebone AI """
if DEVICE == DEVICES[0]:
    print("Running on the BeagleBone AI")
    
    
    ''' code for simulation on your PC '''
elif DEVICE == DEVICES[1]:
    
    # code for simulation on PC
    imu_reading_number = 0
    gps_reading_number = 0
    
    # reading IMU data
    imu_file_data = pd.read_csv("../Data Generation/traj_raster_30mins_20221115_160156.csv").to_numpy();  
    IMU_data = imu_file_data[:, 11:17];
    IMU_t_sec = imu_file_data[:, 0]
        
    # reading GPS data
    gps_file_data = pd.read_csv("../Data Generation/gps_and_barometer_data.csv").to_numpy();
    GPS_data = gps_file_data[:, 1:4]
    GPS_t_sec = gps_file_data[:, 0]
    
    # determine number of data points to read so we don't overflow end of array
    num_points = min(imu_file_data.shape[0], gps_file_data.shape[0])
    
    # get_next_imu_reading()
    #
    # returns the next IMU reading (simulation) """
    def get_next_imu_reading(advance=True):
        global imu_reading_number
        
        # extract the next acceleration and angular rotation
        accel_xyz = IMU_data[imu_reading_number, 0:3]
        gyro_xyz = IMU_data[imu_reading_number, 3:6]
        
        # time step computation (dt)
        if imu_reading_number == 0:
            dt = IMU_t_sec[1] - IMU_t_sec[0]
        else:
            dt = IMU_t_sec[imu_reading_number] - IMU_t_sec[imu_reading_number-1]
            
        if advance:
            imu_reading_number += 1    
        
        return accel_xyz, gyro_xyz, dt
     
    # gps_is_ready()
    #
    # ping the GPS, see if the next data is ready 
    # returns True if ready, False if not
    # in simulation, GPS gets data every 1.0 seconds
    def gps_is_ready():
        global gps_reading_number
        
        if np.isnan(GPS_data[gps_reading_number, 0]):
            gps_reading_number += 1
            #print("FALSE")
            return False
        
        #print("TRUE")
        return True
    
    # get_next_gps_reading()
    #
    # returns the next GPS reading (simulation) """
    def get_next_gps_reading(advance=True):
        global gps_reading_number
        
        # read from the GPS    
        reading = GPS_data[gps_reading_number, 0:3]
        
        # time step computation (dt)
        if gps_reading_number == 0:
            dt = GPS_t_sec[10] - GPS_t_sec[0]
        else:
            dt = GPS_t_sec[imu_reading_number] - GPS_t_sec[imu_reading_number-10]
        
        if advance: # only increment counter if desired
            gps_reading_number += 1
       
        return reading, dt
    
    # get_next_barometer_reading()    
    #
    # returns the next barometer reading (simulation) """
    def get_next_barometer_reading():
        baro = [0, 0, 0]
        return np.array(baro) # TODO
    
    # get_first_quaternion()
    # 
    # determine the initial quaternion of the body, assuming pointing straight up
    # for real-time, this will be implemented with em.lla2quat
    def get_first_quaternion():
        
        return imu_file_data[0, 7:11]
    
    # reset()
    #
    # reset data collection variables
    def reset():
        global imu_reading_number, gps_reading_number
        imu_reading_number = 0
        gps_reading_number = 0
        
    # done()
    #
    # return True if there are no more points to read
    def done():
        #print(gps_reading_number)
        if (imu_reading_number >= num_points) or (gps_reading_number >= num_points):
            #print("EXITING!!")
            return True
        
        return False