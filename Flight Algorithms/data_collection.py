#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler


PURPOSE: Library for data collection from hardware sensors

CLASS: 

FUNCTIONS:
    get_next_imu_reading()
    gps_is_ready() - bool
    get_next_gps_reading()
    get_next_barometer_reading
    get_first_quaternion()
    
"""
import numpy as np

import platform
platform.processor()

#class DataCollector()


"""
IMPORTANT: Set DEVICE variable to DEVICES[1] if running simulation, or DEVICES[0] if running on the actual rocket
"""
DEVICES = ["BEAGLEBONE", "TESTING ON PC"]
DEVICE = DEVICES[1]

if 0: #DEVICE == DEVICES[0]:
    
    import sys
    import time
    sys.path.append('../Sensor Code')
    import readsensors as rs
    import earth_model as em
    
    last_time = time.perf_counter()
    
    """
    All the code that falls in this if statement is for the real-time flight computer. As we get our sensors working, we will add the data here
    """
    
    print("Running on the BeagleBone AI")
    
    def get_next_imu_reading(advance=True):
        """
        gets the next IMU reading 
        
        Arguments:
            - advance: Boolean (optional), if True then data collection advances, if False then on the next call you will get the same data as before
            
        Returns:
            - accel_xyz: 3 x 1 Numpy array
            - gyro_xyz: 3 x 1 Numpy array
            - dt: time step
        """
        global last_time
        
        # extract the next acceleration and angular rotation
        accel_xyz = rs.readaccel()
        gyro_xyz = rs.readgyro()
        
        dt = time.perf_counter() - last_time
        last_time += dt
        
        return accel_xyz, gyro_xyz, dt
    

    def get_next_gps_reading(advance=True):        
        """
        gets the next GPS reading 
        
        Arguments:
            - advance: Boolean (optional), if True then data collection advances, if False then on the next call you will get the same data as before
            
        Returns:
            - reading: 3 x 1 Numpy array [lat, long, atti]
            - dt: time step
        """
        raise NotImplementedError("GPS Reading Not Implemented")
    

    def get_next_barometer_reading():
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
    
    
    def get_first_position():
        
        lla = get_next_gps_reading()
        
        r_ecef = em.lla2ecef(lla)
        v_ecef = np.zeros(3) # initially at rest
        a_ecef = np.zeros(3) # initially no attitude error
        
        return np.concatenate((r_ecef, v_ecef, a_ecef)) # [1527850.153, -4464959.009, 4276353.59, 3.784561502, 1.295026731, -1.11E-16, 0, 0, 0]


    def get_first_quaternion():
        """
        returns the initial state of the Quaternion
        
        Returns:
            - a 4 x 1 quaternion, in the form [qs, qi, qj, qk]
        """
        
        return imu_file_data[0, 7:11]
    

    def reset():
        """
        TODO
        """
        raise NotImplementedError("GPS Reading Not Implemented")
        

    def done():
        """
        TODO
        """
        
        raise NotImplementedError("GPS Reading Not Implemented")
    
