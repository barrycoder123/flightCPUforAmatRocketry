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
from gps_code_modified import read_gps
from readsensors import read_accel, read_gyro, read_baro

class SensorDataCollector:

    def __init__(self):
        
        self.last_time = time.perf_counter()
        
        self.accel_xyz = np.zeros(3)
        self.gyro_xyz = np.zeros(3)
        self.three_baros = np.zeros(3)
    
    
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
        self.accel_xyz = read_accel(self.accel_xyz)
        self.gyro_xyz = read_gyro(self.accel_xyz)
        
        current_time = time.perf_counter()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        return self.accel_xyz, self.gyro_xyz, dt
    

    def get_next_gps_reading(self, advance=True):        
        """
        gets the next GPS reading 
        
        Arguments:
            - advance: Boolean (optional), if True then data collection advances, if False then on the next call you will get the same data as before
            
        Returns:
            - reading: 3 x 1 Numpy array [lat, long, atti]
            - dt: time step
        """
        
        llas = read_gps()
        
        if llas is None:
            return None
        
        return llas[0:3] # ignore satellites
    

    def get_next_barometer_reading(self):
        """
        gets the next barometer reading
        
        Arguments:
            - none
            
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
        
        lla = read_gps()
        while lla is None:
            lla = read_gps()
        
        r_ecef = em.lla2ecef(lla)
        v_ecef = np.zeros(3) # initially at rest
        a_ecef = np.zeros(3) # initially no attitude error
        q_e2b = em.lla2quat(lla)
        
        return np.concatenate((r_ecef, v_ecef, a_ecef)), q_e2b