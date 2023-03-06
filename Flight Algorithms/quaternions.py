#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 18:11:53 2023

@author: zrummler

PURPOSE: Library for quaternions

DATA REPRESENTATION:
    [q_scalar, q_i, q_j, q_k]
    numpy arrays

FUNCTIONS:
    deltaAngleToDeltaQuat
    quat_inv
    quatMultiply
    quat2dcm
    quat2rotmat
    quat2atti
    atti2quat

"""

import numpy as np
import pandas as pd
import earth_model as em
import data_collection as dc


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
    
    mag = np.linalg.norm(dTheta); # norm
    axis = dTheta / mag; # axis of rotation
    
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.html
    theta = mag / 2; # by definition of Quaternion
    q = np.concatenate(([np.cos(theta)], np.sin(theta) * axis));
    
    return q

# quat_inv
#
# Takes the inverse of a quaternion
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
    
    return np.array([w, x, y, z])


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


def rotmat2quat(R):
    """
    Converts a 3x3 rotation matrix to a quaternion.

    Args:
    - R: 3x3 rotation matrix

    Returns:
    - q: 4-dimensional quaternion [q0, q1, q2, q3]
    """

    q = np.zeros(4)

    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2 # S=4*qw 
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S 
        qz = (R[1,0] - R[0,1]) / S
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2 # S=4*qx 
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2 # S=4*qy
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S 
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2 # S=4*qz
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S

    q[0] = qw
    q[1] = qx
    q[2] = qy
    q[3] = qz

    return q


# quat2atti
#
# Determine the attitude error between two quaternions
# Format: [roll_error, pitch_error, yaw_error]
def quat2atti(q_e2b, q_true):
    
    q_error = quatMultiply(quat_inv(q_true), q_e2b)
    
    roll_error = np.arctan2(2*q_error[0]*q_error[1] + 2*q_error[2]*q_error[3], 1 - 2*q_error[1]*q_error[1] - 2*q_error[2]*q_error[2])
    pitch_error = np.arcsin(2*q_error[0]*q_error[2] - 2*q_error[1]*q_error[3])
    yaw_error = np.arctan2(2*q_error[0]*q_error[3] + 2*q_error[1]*q_error[2], 1 - 2*q_error[2]*q_error[2] - 2*q_error[3]*q_error[3])

    atti_error = [roll_error, pitch_error, yaw_error]
    
    return np.array(atti_error)


# atti2quat
#
# Determine true quaternion from attitude error and measured quaternion
def atti2quat2(atti_error, q_true):
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

def atti2quat(atti):
    roll, pitch, yaw = atti
    cy = np.cos(yaw*0.5)
    sy = np.sin(yaw*0.5)
    cp = np.cos(pitch*0.5)
    sp = np.sin(pitch*0.5)
    cr = np.cos(roll*0.5)
    sr = np.sin(roll*0.5)
    
    qw = cy*cp*cr + sy*sp*sr
    qx = cy*cp*sr - sy*sp*cr
    qy = sy*cp*sr + cy*sp*cr
    qz = sy*cp*cr - cy*sp*sr
    
    return np.array([qw, qx, qy, qz])


if __name__ == "__main__":
    
    q_true = np.array([-1 , 120231, 234, 2342])
    
    rot = quat2rotmat(q_true)
    q_2 = rotmat2quat(rot)
    
    print(q_true, q_2)
    
    