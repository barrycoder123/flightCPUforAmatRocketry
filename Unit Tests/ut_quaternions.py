#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:37:36 2023

@author: zrummler
"""

import sys
import numpy as np

sys.path.append('../Flight Algorithms')

import earth_model as em

# X-axis: points towards intersection of equator and prime meridian
# Y-axis: points towards equator and 90-degrees meridian
# Z-axis: points through the earth's north pole

llas = np.vstack([
        [90, 0, 0],
        [-90, 0, 0], # south pole
        [0, 0, 0], # gulf of guinea, africa (null island)
        [0, 180, 0], # pacific ocean
        [0, 90, 0], # indian ocean
        [0, -90, 0]]) # atlantic ocean

xyzs = np.array([
        [0, 0, em.b], # north pole
        [0, 0, -em.b], # south pole
        [em.a, 0, 0], # gulf of guinea, africa
        [-em.a, 0, 0], # pacific ocean
        [0, em.a, 0], # indian ocean
        [0, -em.a, 0]]) # atlantic ocean

def unit_test_lla2ecef():
        
    for i in range(len(llas)):
        lla = llas[i] # north pole
        xyz = em.lla2ecef(lla)
        if not np.allclose(xyzs[i].reshape(-1,1), xyz):
            print("lla2ecef test failed!")
            print(xyzs[i].reshape(-1,1))
            print(xyz)
            return
        
        
    print("lla2ecef all passed!")
    
def unit_test_ecef2lla():
    
    for i in range(len(xyzs)):
        xyz = xyzs[i]
        lla = em.ecef2lla(xyz)
        if not np.allclose(llas[i].reshape(-1,1), lla):
            print("ecef2lla test failed!")
            print(llas[i].reshape(-1,1))
            print(lla)
            return

    print("ecef2lla all passed!")  
   

def unit_test_xyz2grav():
    
    for i in range(len(xyzs)):
        xyz = xyzs[i]
        grav = em.xyz2grav(xyz)
        print(grav)
    
def unit_test_lla2quat():
    
    for i in range(len(llas)):
        lla = llas[i] # north pole
        quat = em.lla2quat(lla)
        print(quat)
        

def unit_test_lla2quat():
    
    for i in range(len(llas)):
        lla = llas[i]
        quat = em.lla2quat(lla)
        print(quat)

    
if __name__ == "__main__":
    
    
    #unit_test_lla2ecef()
    #unit_test_ecef2lla()
    
    #unit_test_xyz2grav()
    
    unit_test_lla2quat()
