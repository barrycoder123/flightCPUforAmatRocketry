#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 12:25:45 2023

@author: zrummler
"""

import os
import numpy as np
import pandas as pd
from data_collection_wrapper import DataCollector

class FileDataCollector(DataCollector):
    
    def __init__(self):

        # code for simulation on PC
        self.imu_reading_number = 0
        self.gps_reading_number = 0
        self.baro_reading_number = 0
        
        # reading IMU data
        file_path = "../Test Data/traj_raster_30mins_20221115_160156.csv"
        self.imu_file_data = pd.read_csv(file_path).to_numpy();  
        self.IMU_data = self.imu_file_data[:, 11:17];
        self.IMU_t_sec = self.imu_file_data[:, 0]
            
        # reading GPS data
        file_path = "../Test Data/gps_and_barometer_data.csv"
        self.gps_file_data = pd.read_csv(file_path).to_numpy();
        self.GPS_data = self.gps_file_data[:, 1:4]
        self.baro_data = self.gps_file_data[:, 4:7]
        self.GPS_t_sec = self.gps_file_data[:, 0]
        
        # determine number of data points to read so we don't overflow end of array
        self.num_points = min(self.imu_file_data.shape[0], self.gps_file_data.shape[0])
    

    def get_next_imu_reading(self):
        """
        gets the next IMU reading 

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
        
        self.imu_reading_number += 1
        
        return accel_xyz, gyro_xyz, dt
    

    def get_next_gps_reading(self):        
        """
        gets the next GPS reading 
        
        Returns:
            - lla: 3 x 1 Numpy array [lat, long, atti]
            
        Notes:
            - Returns None if no reading can be made
        """
        
        if np.isnan(self.GPS_data[self.gps_reading_number, 0]):
            self.gps_reading_number += 1
            return None
        
        # read from the GPS    
        reading = self.GPS_data[self.gps_reading_number, 0:3]
        
        self.gps_reading_number += 1
        
        return reading
    

    def get_next_barometer_reading(self):
        """
        gets the next barometer reading
            
        Returns:
            - reading: (3,1) or (3,) numpy array of three readings [baro1, baro2, baro3]
        """
        
        reading = self.baro_data[self.baro_reading_number, 0:3]
        
        self.baro_reading_number += 1
        
        return reading
    
    
    def get_initial_state_and_quaternion(self):
        """
        returns the initial state vector and initial Quaternion
        
        Returns:
            - state vector (9,)
            - quaternion (4,)
        """
        
        r_ecef = self.imu_file_data[0, 1:4].flatten()
        v_ecef = self.imu_file_data[0, 4:7].flatten()
        a_ecef = np.zeros(3)
        
        return np.concatenate((r_ecef, v_ecef, a_ecef)), self.imu_file_data[0, 7:11] 

    def start_timer(self):
        pass