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
sys.path.append('../Data Collection')

import earth_model as em
from read_gps import read_gps_lla
from read_imu import calibrate_imu, read_imu_accel, read_imu_gyro
from read_baro import read_baro_alt
from data_collection_wrapper import DataCollector

NUM_SATELLITES = 8 # number of satellites requested for each GPS fix. Variable

# hard-coded initial conditions
GPS_AT_574 = np.array([42.403061, -71.113635, 40.0])
Q_E2B_CAMBRIDGE = np.array([0.901797015, -0.39979036, -0.066508461,-0.150021455])

class AvgBuf():
    """
    Class allows us to take a moving average of previous accel or gyro readings
    """
    
    def __init__(self, read_func):
        
        self.N = 10
        
        self.prevAccel = np.zeros([3, self.N])
        self.index = 0
        self.movingXYZ = np.zeros(3)
        
        i = 0
        while i < self.N:
            reading = read_func()
            if reading is not None:
                self.prevAccel[:,i] = reading
                i += 1       
        print(self.prevAccel)


    def update(self, xyz):

        #print(self.prevAccel.shape)
        self.prevAccel = np.append(self.prevAccel,xyz.reshape(-1,1),axis=1)
        #print(self.prevAccel.shape)
        self.prevAccel = np.delete(self.prevAccel,0,axis=1)
        #print(self.prevAccel.shape)
    
    def getAvg(self):
        return np.mean(self.prevAccel,axis=1)


class SensorDataCollector(DataCollector):

    def __init__(self, choose_points=True):
        
        """
        Initializes arrays and waits for GPS to warm up
        """

        if choose_points == True:
            print("Please choose the number of data points you'd like to collect")
            self.num_points = int(input(">> "))
        
        #calibrate_imu()

        calibrate_imu()

        self.accel_buf = AvgBuf(read_imu_accel)
        self.gyro_buf = AvgBuf(read_imu_gyro)
        
           
    
    def start_timer(self):
        """
        start the data collector timer
        
        No arguments, no return value
        """
        
        t_initial = time.monotonic()
        self.last_imu_time = t_initial


    def get_next_imu_reading(self):
        """
        gets the next IMU reading  (acceleration, angular rotation)
        Also returns time step

        Returns:
            - accel_xyz: 3 x 1 Numpy array
            - gyro_xyz: 3 x 1 Numpy array
            - dt: time step
        """

        # extract the next acceleration and angular rotation
        time.sleep(0.05)
        accel_xyz = read_imu_accel()
        gyro_xyz = read_imu_gyro()

        if accel_xyz is None:
            # do the moving average
            accel_xyz = self.accel_buf.getAvg()
            print(accel_xyz)
        else:
            self.accel_buf.update(accel_xyz)
        
        if gyro_xyz is None:
            gyro_xyz = self.gyro_buf.getAvg()
            print(gyro_xyz)
        else:
            self.gyro_buf.update(gyro_xyz)

        
        # determine change in time (seconds) between last and current IMU read
        current_time = time.monotonic()
        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time
        
        #self.gyro_xyz = np.zeros((3))
        #self.accel_xyz = np.array([0, 0, 9.81])
        
        return accel_xyz, gyro_xyz, dt
    

    def get_next_gps_reading(self):        
        """
        gets the next GPS reading 
        
        Returns:
            - lla: 3 x 1 Numpy array [lat, long, atti]
            
        Notes:
            - Returns None, 0 if no reading can be made
        """
         
        # update every 1.0 seconds
        lla = read_gps_lla(NUM_SATELLITES, 1.0)
        
        if lla is not None:
            lla = np.array(lla)
            print(lla)
        
        return lla
    

    def get_next_barometer_reading(self):
        """
        gets the next barometer reading
            
        Returns:
            - reading: single barometer reading in altitude
            
        Note:
            - does not work
        """

        return read_baro_alt()
    
    
    def get_initial_state_and_quaternion(self):
        """
        returns the initial state vector and initial Quaternion
        
        Returns:
            - state vector (9,)
            - quaternion (4,)
        """
        
        # wait for GPS to warm up, then save that reading
        lla = None
        #lla = GPS_AT_574
        while lla is None:
            lla = read_gps_lla(NUM_SATELLITES)
        lla = np.array(lla)
        print("Initial LLA:",lla)

        q_e2b = Q_E2B_CAMBRIDGE

        r_ecef = em.lla2ecef(lla).flatten()
        v_ecef = np.zeros(3) # initially at rest
        a_ecef = np.zeros(3) # initially no accel

        return np.concatenate((r_ecef, v_ecef, a_ecef)), q_e2b
