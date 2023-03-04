#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 18:11:53 2023

@author: zrummler
"""

import numpy as np

def norm(v):
    sum_squares = sum([vi**2 for vi in v])
    return np.sqrt(sum_squares)

# deltaAngleToDeltaQuat
#
# Converts a vector of rotations to a Quaternion
#     Parameters
#     ----------
#     dTheta : (3,) vector
#       delta angles [rad]
#
#     Returns
#     -------
#     q : (4,) vector
#       Quaternion
def deltaAngleToDeltaQuat(dTheta):
    
    mag = norm(dTheta); # norm
    axis = dTheta / mag; # axis of rotation
    
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.html
    theta = mag / 2; # by definition of Quaternion
    q = np.concatenate(([np.cos(theta)], np.sin(theta) * axis));
    
    return q

# quatMultiply
#
# Multiply two quaternions  
def quatMultiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return [w, x, y, z]

# quat2dcm
#
# Convert quaternion to a 3x3 discrete cosine matrix (dcm)
def quat2dcm(q):
    q0, q1, q2, q3 = q
    dcm = [[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
           [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
           [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]]
    return np.array(dcm)