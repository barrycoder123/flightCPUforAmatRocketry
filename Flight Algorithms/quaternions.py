#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 18:11:53 2023

@author: zrummler

Library for quaternions

Quaternion Format: [scalar, i, j, k]

"""


import numpy as np
import pandas as pd
import earth_model as em
import data_collection as dc

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
# Credit: Tyler Klein
def deltaAngleToDeltaQuat(dTheta):
    
    mag = norm(dTheta); # norm
    axis = dTheta / mag; # axis of rotation
    
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.html
    theta = mag / 2; # by definition of Quaternion
    q = np.concatenate(([np.cos(theta)], np.sin(theta) * axis));
    
    return q

def quat_inv(q):
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    q_norm = np.linalg.norm(q)
    q_inv = q_conj / q_norm**2
    return q_inv


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


# quat2rotmatrix
#
# Convert quaternion to rotation matrix.
def quat2rotmat(q):
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
        [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
        [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]])


""" HAVE NOT TESTED """
# quat2atti
#
# Determine the attitude error between two quaternions
def quat2atti(q_e2b, q_true):
    
    q_error = quatMultiply(quat_inv(q_true), q_e2b)
    
    roll_error = np.arctan2(2*q_error[0]*q_error[1] + 2*q_error[2]*q_error[3], 1 - 2*q_error[1]*q_error[1] - 2*q_error[2]*q_error[2])
    pitch_error = np.arcsin(2*q_error[0]*q_error[2] - 2*q_error[1]*q_error[3])
    yaw_error = np.arctan2(2*q_error[0]*q_error[3] + 2*q_error[1]*q_error[2], 1 - 2*q_error[2]*q_error[2] - 2*q_error[3]*q_error[3])

    atti_error = np.array([roll_error, pitch_error, yaw_error])
    
    return atti_error

""" HAVE NOT TESTED """
# atti2quat
#
# Determine true quaternion from attitude error and measured quaternion
def atti2quat(atti_error, q_true):
    roll_error, pitch_error, yaw_error = atti_error
    sin_roll = np.sin(roll_error / 2)
    cos_roll = np.cos(roll_error / 2)
    sin_pitch = np.sin(pitch_error / 2)
    cos_pitch = np.cos(pitch_error / 2)
    sin_yaw = np.sin(yaw_error / 2)
    cos_yaw = np.cos(yaw_error / 2)
    q_error = np.array([cos_roll*cos_pitch*cos_yaw + sin_roll*sin_pitch*sin_yaw,
                        sin_roll*cos_pitch*cos_yaw - cos_roll*sin_pitch*sin_yaw,
                        cos_roll*sin_pitch*cos_yaw + sin_roll*cos_pitch*sin_yaw,
                        cos_roll*cos_pitch*sin_yaw - sin_roll*sin_pitch*cos_yaw])
    q_est = quatMultiply(q_error, q_true)
    return q_est



if __name__ == "__main__":
    
    q_true = np.array([1, 2, 3, 4])
    q_e2b = np.array([1.04, 2.34, 3.54, 4.12])
    
    atti_error = quat2atti(q_e2b, q_true)
    q_true_2 = atti2quat(atti_error, q_e2b)
    
    print(q_true, q_true_2)
    
    