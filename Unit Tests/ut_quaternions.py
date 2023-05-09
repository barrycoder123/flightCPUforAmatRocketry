#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:37:36 2023

@author: zrummler
"""

import sys

sys.path.append('../Flight Algorithms')

import quaternions as qt


def unit_test_quat_init():
    """
    Just trying stuff out. not needed in our implementation
    """
    
    q_imu = [0.99993896, 0.00927734, -0.00268555, 0]
    q_cambridge = [0.901797015, -0.39979036, -0.066508461, -0.150021455]
    
    print(qt.quatMultiply(q_imu, qt.quat_inv(q_cambridge)))
    print(qt.quatMultiply(qt.quat_inv(q_imu), q_cambridge))
    
    print(qt.quatMultiply(q_cambridge, qt.quat_inv(q_imu)))
    print(qt.quatMultiply(qt.quat_inv(q_cambridge), q_imu))
    
if __name__ == "__main__":
    
    unit_test_quat_init()