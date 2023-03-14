#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 13 18:45:51 2023

@author: zrummler
"""

import sys
import numpy as np

sys.path.append('../Flight Algorithms')

import data_collection as dc

if __name__ == "__main__":
    
    dc.reset()
    
    n = 100
    
    baro_readings = np.zeros((n,3))
    gps_readings = np.zeros((n,3))
    
    for i in range(n):
        
        baro_readings[i,:] = dc.get_next_barometer_reading()
        if dc.gps_is_ready():
            idx = i//10
            lla, dt = dc.get_next_gps_reading()
            #print(lla)
            gps_readings[i,:] = lla
            
    print("Barometer:", np.std(baro_readings, axis=0))
    print("GPS:", np.std(gps_readings,axis=0))
        
    