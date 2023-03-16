#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler


PURPOSE: 
- Wrapper for data collection class
- Selects the FileDataCollector sublass if you're on a PC
- Selects the SensorDataCollector subclass if you're on the actual nav CPU

FUNCTIONS:
    get_next_imu_reading()
    get_next_gps_reading()
    get_next_barometer_reading()
    get_initial_state_and_quaternion()
    
"""

import platform
import importlib

class DataCollector:
    
    def __init__(self):
        pass
    
    def create(self):
        
        # For Zack on a Mac
        if platform.processor() == 'i386':
            return self.__file_data_collector()
        
        # Print welcome messages and instructions
        print("Welcome to the Data Collection module!") 
        print("You are running this code on an unrecognized computer. It's likely that you're running on the BeagleBoard for the first time, or you're on a new PC.\n")
        print("\tSelect (1) if you are developing/debugging on a PC (i.e., using test data from a file)\n")
        print("\tSelect (2) if you are debugging on the Flight Computer (i.e., using hardware sensors)\n")
        
        # Take input until you get 1 or 2
        val = 0
        while (val != 1) and (val != 2):
            val = int(input(">> "))
        
        # Choose subclass based on selection
        if val == 1:
            return self.__file_data_collector()
        if val == 2:
            return self.__sensor_data_collector()
        else:
            print("Unrecognized input, try again...")
            assert(False)
            
    def get_next_baro_reading(self):
        raise NotImplementedError("Did you forget to call DataCollector().create() ?")
    
    def get_next_imu_reading(self):
        raise NotImplementedError("Did you forget to call DataCollector().create() ?")
    
    def get_next_gps_reading(self):
        raise NotImplementedError("Did you forget to call DataCollector().create() ?")
        
    def get_initial_state_and_quaternion(self):
        raise NotImplementedError("Did you forget to call DataCollector().create() ?")
           
    @classmethod
    def __file_data_collector(self):
        from file_data_collector import FileDataCollector
        return FileDataCollector()
    
    @classmethod
    def __sensor_data_collector(self):
        from sensor_data_collector import SensorDataCollector
        return SensorDataCollector()

