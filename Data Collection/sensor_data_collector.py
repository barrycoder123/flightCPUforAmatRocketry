#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 13:06:09 2023

@author: zrummler
"""

import sys
import time
import numpy as np

sys.path.append('../Flight Algorithms')

import earth_model as em
from gps_code_modified import read_gps, gps
from readsensors import read_accel, read_gyro, read_baro
from data_collection_wrapper import DataCollector

NUM_SATELLITES = 8 # number of satellites required for each GPS fix

class SensorDataCollector(DataCollector):

    def __init__(self):
        
        """
        Initializes arrays and waits for GPS to warm up
        """
        gps.readline() #read and ignore first line
        print("Please choose the number of data points you'd like to collect")
        self.num_points = int(input(">> "))
        
        self.accel_xyz = np.zeros(3)
        self.gyro_xyz = np.zeros(3)
        self.three_baros = np.zeros(3)
        
        # Wait for GPS to warm up
        lla = None
        while lla is None:
            lla = read_gps(NUM_SATELLITES)
        print("Exited while loop")
    
    def get_next_imu_reading(self):
        """
        gets the next IMU reading 

        Returns:
            - accel_xyz: 3 x 1 Numpy array
            - gyro_xyz: 3 x 1 Numpy array
            - dt: time step
        """
        
        # extract the next acceleration and angular rotation
        self.accel_xyz = read_accel(self.accel_xyz)
        self.gyro_xyz = read_gyro(self.accel_xyz)
        
        current_time = time.perf_counter()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        return self.accel_xyz, self.gyro_xyz, dt
    

    def get_next_gps_reading(self):        
        """
        gets the next GPS reading 
        
        Returns:
            - lla: 3 x 1 Numpy array [lat, long, atti]
            
        Notes:
            - Returns None, 0 if no reading can be made
        """
        
        lla = read_gps(NUM_SATELLITES)
        
        return lla
    

    def get_next_barometer_reading(self):
        """
        gets the next barometer reading
            
        Returns:
            - reading: (3,1) or (3,) numpy array of three readings [baro1, baro2, baro3]
        """
        
        self.three_baros[0] = read_baro(self.three_baros[0])
        self.three_baros[1] = read_baro(self.three_baros[1])
        self.three_baros[2] = read_baro(self.three_baros[2])

        return self.three_baros
    
    
    def get_initial_state_and_quaternion(self):
        """
        returns the initial state vector and initial Quaternion
        
        Returns:
            - state vector (9,)
            - quaternion (4,)
        """
        
        # read GPS if lla is passed as none
        lla = None
        while lla is None:
            lla = read_gps(NUM_SATELLITES)
        
        r_ecef = em.lla2ecef(lla)
        v_ecef = np.zeros(3) # initially at rest
        a_ecef = np.zeros(3) # initially no attitude error
        q_e2b = em.lla2quat(lla)
        
        self.last_time = time.perf_counter() # this could be a problem
        
        return np.concatenate((r_ecef, v_ecef, a_ecef)), q_e2b
