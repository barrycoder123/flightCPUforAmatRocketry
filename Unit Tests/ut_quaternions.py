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
    
    # Test case 1: q1 and q2 are identical
    q1 = np.array([1, 0, 0, 0])
    q2 = np.array([1, 0, 0, 0])
    q_error_expected = np.array([0, 0, 0, 0])
    q_error = qt.quat_error(q1, q2)
    assert np.allclose(q_error, q_error_expected)

    # Test case 2: q1 and q2 differ by 90-degree rotation about Z-axis
    q1 = np.array([1, 0, 0, 0])
    q2 = np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])
    q_error_expected = np.array([0, 0, 0, np.sqrt(2)])
    q_error = qt.quat_error(q1, q2)
    print(q_error)
    assert np.allclose(q_error, q_error_expected)

    # Test case 3: Convert error quaternion back to original quaternion
    q1 = np.array([1, 0, 0, 0])
    q_error = np.array([0, 0, 0, np.sqrt(2)])
    q2_expected = np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])
    q2 = qt.quat_from_err(q1, q_error)
    assert np.allclose(q2, q2_expected)

    
    
    print("All unit tests passed.")
    
if __name__ == "__main__":
    
    #unit_test_quat2atti()
    
    unit_test_quat_error()
