#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 13:06:09 2023

@author: zrummler
"""

import sys
import time
import numpy as np
import readsensors as rs

sys.path.append('../Flight Algorithms')

import earth_model as em
import quaternion as qt

class SensorDataCollector:

    def __init__(self):
        
        self.last_time = time.perf_counter()
    
    """
    All the code that falls in this if statement is for the real-time flight computer. As we get our sensors working, we will add the data here
    """
    
    print("Running on the BeagleBone AI")
    
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
        accel_xyz = rs.readaccel()
        gyro_xyz = rs.readgyro()
        
        dt = time.perf_counter() - self.last_time
        self.last_time += dt
        
        return accel_xyz, gyro_xyz, dt
    

    def get_next_gps_reading(self, advance=True):        
        """
        gets the next GPS reading 
        
        Arguments:
            - advance: Boolean (optional), if True then data collection advances, if False then on the next call you will get the same data as before
            
        Returns:
            - reading: 3 x 1 Numpy array [lat, long, atti]
            - dt: time step
        """
        raise NotImplementedError("GPS Reading Not Implemented")
    

    def get_next_barometer_reading(self):
        """
        gets the next barometer reading
        
        Arguments:
            - none
            
        Returns:
            - reading: (3,1) or (3,) numpy array of three readings [baro1, baro2, baro3]
        """
        
        three_baros = np.zeros(3)
        
        three_baros[0] = rs.readbaro()
        three_baros[1] = rs.readbaro()
        three_baros[2] = rs.readbaro()

        return three_baros
    
    
    def get_first_position(self):
        
        lla = self.get_next_gps_reading()
        
        r_ecef = em.lla2ecef(lla)
        v_ecef = np.zeros(3) # initially at rest
        a_ecef = np.zeros(3) # initially no attitude error
        
        return np.concatenate((r_ecef, v_ecef, a_ecef)) # [1527850.153, -4464959.009, 4276353.59, 3.784561502, 1.295026731, -1.11E-16, 0, 0, 0]


    def get_first_quaternion(self):
        """
        returns the initial state of the Quaternion
        
        Returns:
            - a 4 x 1 quaternion, in the form [qs, qi, qj, qk]
        """
        
        return qt.lla2qu