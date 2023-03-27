#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 27 11:32:11 2023

@author: zrummler

FlightCPUStateMachine class
"""

import sys
import time

sys.path.append('../Data Collection')
sys.path.append('../Data Logging')

import kalman as kf
import data_logging_wrapper as dl
import data_collection_wrapper as dc

class StateMachine:
    
    def launch_detect(self):
        print("IN LD")
        
        #1: Initialize GPS, IMU, and Barometer objects. At this step the barometer should read the sea level barometric pressure from the config file.
        self.data_collector = dc.DataCollector().create()
        
        #2: Wait 30 seconds. Allows GPS and IMU to warm up.
        wait_time = 0.1 # make this 30
        time.sleep(wait_time)
        
        #3: Continually read from the GPS until a reading with at least 8 satellites is made. Once a reading with 8 satellites is made, save that reading [Lat, Lon, Alt]
        
        
        #4: Initialize attitude with GPS and IMU reading. Currently: get_initial_state_and_quaternion() in Data collection. Need to re-write to take in LLA from valid GPS update (8+ satellites, and freshly calibrated and warmed up IMU).
        #5: Instantiate buffer file that data will be logged to. Populate first line of file with state vector initialization.
    
        
    def launch_detect_pyro_arm(self):
        print("IN LD PA")
        
        #1: Wait for pyro arm switch to be ON. Pyro arm can happen at any point before step 5. Step of 5 launch detect must be completed before LD-PA. There should be some boolean which stores pyro arm state. Pyro will be armed by a hardware switch.
        #2: If step 1 is complete, read IMU and state vector continuously. If either the IMU detects an acceleration along Z axis of over (insert) for at least 3 samples or the velocity of the state vector is greater than (insert), Active Flight mode is triggered. (This condition needs to be tested and can be changed)
    
        
    def active_flight(self):
        print("IN AF")
        
        self.data_logger = dl.DataLogger().create()
        
        #1: Continuously log data to file buffer. Continuously monitor state vector Z velocity (ECEF) and altitude. If computer is in fixed altitude operator, deploy chute once that altitude is detected. If computer is in autodetect mode, fire chutes after .5 seconds of 3 state vector reading of Z velocity (ECEF) less than 0.5 m/s (test). 
        #2: If an acceleration of less than 0.5 m/s is detected for more that 5 seconds, end data logging, end program execution and save data file.
    
        

if __name__ == "__main__":
    
    # ------- state 1 ------- #
    launch_detect()
    
    # ------- state 2 ------- #
    launch_detect_pyro_arm()
    
    # ------- state 3 ------- #
    active_flight()