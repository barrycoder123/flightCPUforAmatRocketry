#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:37:36 2023

@author: zrummler
"""

import sys
import numpy as np

sys.path.append('../FLight Algorithms')

import quaternions as qt

q_inputs = np.array([
    [0.3, 0.4, 0.5, 0.6],
    [0.0, 0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0]])

def unit_test_quat2atti():
    
    print("Testing quat2atti and atti2quat:")
    
    # Test quaternion error [0.3, 0.4, 0.5, 0.6]
    for i in range(len(q_inputs)):
        q_error = np.array(q_inputs[i])
        #q_error /= np.linalg.norm(q_error)
        atti_error = qt.quat2atti(q_error)
        q_error_reconstructed = qt.atti2quat(atti_error)
        assert np.allclose(q_error, q_error_reconstructed)
    
    
    print("All unit tests passed.")
    
def unit_test_quat_error():
    
    print("Testing quat_error")
    
    # Test quaternion error [0.3, 0.4, 0.5, 0.6]
    for i in range(len(q_inputs)):
        for j in range(len(q_inputs)):
            q1 = np.array(q_inputs[i])
            q2 = np.array(q_inputs[j])
            q_error = qt.quat_error(q1, q2)
            #print(q1, q2)
            #print("error: ", q_error)
            q2_reconstructed = qt.quat_error_rev(q_error, q1)
            #print(q2, q2_reconstructed)
            assert np.allclose(q2, q2_reconstructed)
    
    
    print("All unit tests passed.")
    
if __name__ == "__main__":
    
    #unit_test_quat2atti()
    
    unit_test_quat_error()
    
    q1 = np.array([1, 0, 0, 0])
    q2 = np.array([-1, 0, 0, 0])
    # Compute the error between q1 and q2
    q_error = np.multiply(q2, q1.conj())
    
    # Compute the norm of the error vector
    error_norm = np.linalg.norm(q_error)
    
    # Scale the error vector by 2 to obtain the error between q1 and q2
    q_error_scaled = 2 * q_error
    
    print(q_error_scaled)