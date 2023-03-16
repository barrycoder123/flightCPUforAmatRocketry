#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 12:25:45 2023

@author: zrummler
"""

import numpy as np
import pandas as pd

class FileDataCollector():
    
    def __init__(self):
    
        """
        All the code that falls below this else statement is for simulation purposes only. Instead of getting data from sensors, the simulation gets data from the datafiles
        """
        
        # code for simulation on PC
        self.imu_reading_number = 0
        self.gps_reading_number = 0
        self.baro_reading_number = 0
        
        # reading IMU data
        self.imu_file_data = pd.read_csv("../Data Generation/traj_raster_30mins_20221115_160156.csv").to_numpy();  
        self.IMU_data = self.imu_file_data[:, 11:17];
        self.IMU_t_sec = self.imu_file_data[:, 0]
            
        # reading GPS data
        self.gps_file_data = pd.read_csv("../Data Generation/gps_and_barometer_data.csv").to_numpy();
        self.GPS_data = self.gps_file_data[:, 1:4]
        self.baro_data = self.gps_file_data[:, 4:7]
        self.GPS_t_sec = self.gps_file_data[:, 0]
        
        # determine number of data points to read so we don't overflow end of array
        self.num_points = min(self.imu_file_data.shape[0], self.gps_file_data.shape[0])
    

    def get_next_imu_reading(self, advance=True):
        """
        gets the next IMU reading 
        
        Arguments:
            - advance: Boolean (optional), if True then data collection advances, if False then on the next call you will get the same data as before
            
        Returns:
            - accel_xyz: 3 x 1 Numpy array
            - gyro_xyz: 3 x 1 Numpy array
            - dt: time step
        """
        
        # extract the next acceleration and angular rotation
        accel_xyz = self.IMU_data[self.imu_reading_number, 0:3]
        gyro_xyz = self.IMU_data[self.imu_reading_number, 3:6]
        
        # time step computation (dt)
        if self.imu_reading_number == 0:
            dt = self.IMU_t_sec[1] - self.IMU_t_sec[0]
        else:
            dt = self.IMU_t_sec[self.imu_reading_number] - self.IMU_t_sec[self.imu_reading_number-1]
            
        if advance:
            self.imu_reading_number += 1    
        
        return accel_xyz, gyro_xyz, dt
     

    def gps_is_ready(self):
        """
        returns True if the GPS has a new data value to return, or False if not
        for simulation purposes, GPS gets data every 1.0 seconds
        """
        self.gps_reading_number
        
        if np.isnan(self.GPS_data[self.gps_reading_number, 0]):
            self.gps_reading_number += 1
            #print("FALSE")
            return False
        
        return True
    

    def get_next_gps_reading(self, advance=True):        
        """
        gets the next GPS reading 
        
        Arguments:
            - advance: Boolean (optional), if True then data collection advances, if False then on the next call you will get the same data as before
            
        Returns:
            - reading: 3 x 1 Numpy array [lat, long, atti]
            - dt: time step
        """
        
        if not self.gps_is_ready():
            return None, -1
        
        # read from the GPS    
        reading = self.GPS_data[self.gps_reading_number, 0:3]
        
        # time step computation (dt)
        if self.gps_reading_number == 0:
            dt = self.GPS_t_sec[10] - self.GPS_t_sec[0]
        else:
            dt = self.GPS_t_sec[self.gps_reading_number] - self.GPS_t_sec[self.gps_reading_number-10]
        
        if advance: # only increment counter if desired
            self.gps_reading_number += 1
       
        return reading, dt
    

    def get_next_barometer_reading(self):
        """
        gets the next barometer reading
        
        Arguments:
            - none
            
        Returns:
            - reading: (3,1) or (3,) numpy array of three readings [baro1, baro2, baro3]
        """
        
        reading = self.baro_data[self.baro_reading_number, 0:3]
        
        self.baro_reading_number += 1
        
        return reading
    
    
    def get_first_position(self):
        
        r_ecef = self.imu_file_data[0, 1:4].flatten()
        v_ecef = self.imu_file_data[0, 4:7].flatten()
        a_ecef = np.zeros(3)
        
        return np.concatenate((r_ecef, v_ecef, a_ecef)) # [1527850.153, -4464959.009, 4276353.59, 3.784561502, 1.295026731, -1.11E-16, 0, 0, 0]


    def get_first_quaternion(self):
        """
        returns the initial state of the Quaternion
        
        Returns:
            - a 4 x 1 quaternion, in the form [qs, qi, qj, qk]
        """
        return self.imu_file_data[0, 7:11]
    

    def reset(self):
        """
        resets data collection variables and counters
        
        """

        self.imu_reading_number = 0
        self.gps_reading_number = 0
        self.baro_reading_number = 0