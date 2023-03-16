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
    
    
    

    
