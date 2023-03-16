#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 18:11:53 2023

@author: zrummler

PURPOSE: Library for quaternions. I'd rather implement than use a pre-made quaternion library.

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
    
SOURCE:
    ChatGPT was instrumental in making this library work

"""

import numpy as np


def deltaAngleToDeltaQuat(dTheta):
    """
    Converts a vector of rotations to a Quaternion

    Args:
    - dTheta: 3-dimensional vector, delta angles, in radians

    Returns:
    - q_inv: 4-dimensional quaternion [qs, qi, qj, qk]
    
    Credit: Tyler Klein
    """
    mag = np.linalg.norm(dTheta)  # norm

    if mag < 1e-10:
        return np.array([1, 0, 0, 0])

    axis = dTheta / mag  # axis of rotation

    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.html
    theta = mag / 2  # by definition of Quaternion
    q = np.concatenate(([np.cos(theta)], np.sin(theta) * axis))

    return q


def quat_inv(q):
    """
    Takes the inverse of a quaternion

    Args:
    - q: 4-dimensional quaternion [qs, qi, qj, qk]

    Returns:
    - q_inv: 4-dimensional quaternion [qs, qi, qj, qk]
    """
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    q_norm = np.linalg.norm(q)
    q_inv = q_conj / q_norm ** 2
    return q_inv


def quatMultiply(q1, q2):
    """
    Multiplies two quaternions together

    Args:
    - q1, q2: two 4-dimensional quaternion [qs, qi, qj, qk]

    Returns:
    - q: 4-dimensional quaternion [qs, qi, qj, qk]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def quat2dcm(q):
    """
    Converts a quaternion to a 3x3 discrete cosine matrix (dcm)

    Args:
    - q: 4-dimensional quaternion [qs, qi, qj, qk]

    Returns:
     - R: 3x3 dcm matrix  
    """
    q0, q1, q2, q3 = q
    dcm = [[q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
           [2 * (q1 * q2 + q0 * q3), q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2, 2 * (q2 * q3 - q0 * q1)],
           [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2]]
    return np.array(dcm)

'''
def quat2rotmat(q):
    """
    Converts a quaternion to a 3x3 rotation matrix

    Args:
    - q: 4-dimensional quaternion [qs, qi, qj, qk]

    Returns:
     - R: 3x3 rotation matrix   
    """
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2 * q2 ** 2 - 2 * q3 ** 2, 2 * q1 * q2 - 2 * q0 * q3, 2 * q1 * q3 + 2 * q0 * q2],
        [2 * q1 * q2 + 2 * q0 * q3, 1 - 2 * q1 ** 2 - 2 * q3 ** 2, 2 * q2 * q3 - 2 * q0 * q1],
        [2 * q1 * q3 - 2 * q0 * q2, 2 * q2 * q3 + 2 * q0 * q1, 1 - 2 * q1 ** 2 - 2 * q2 ** 2]])


def rotmat2quat(R):
    """
    Converts a 3x3 rotation matrix to a quaternion.

    Args:
    - R: 3x3 rotation matrix

    Returns:
    - q: 4-dimensional quaternion [qs, qi, qj, qk]
    """
    q = np.zeros(4)

    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    q[0] = qw
    q[1] = qx
    q[2] = qy
    q[3] = qz

    return q
'''
'''
def quat_error(q1, q2):
    """
    Computes the error between two quaternions

    Args:
    - q1, q2: 4-dimensional quaternions [qs, qi, qj, qk]

    Returns:
    - q: 4-dimensional quaternion [qs, qi, qj, qk]
    """

    if np.allclose(q1, q2):
        return np.array([0, 0, 0, 0])

    return quatMultiply(q2, quat_inv(q1))

    # R1 = quat2rotmat(q1)
    # R2 = quat2rotmat(q2)
    # R12 = np.dot(R1, R2.T)
    # tr = np.trace(R12)
    # costh = (tr - 1.0) / 2.0
    # sinth = np.sqrt(1.0 - costh**2)
    # sinth = np.maximum(sinth, np.finfo(float).eps)
    # theta = np.arccos(costh) * 2.0
    # q_error = np.array([np.sin(theta / 2.0), 0.0, 0.0, np.cos(theta / 2.0)])
    # return q_error

    # Scale the error vector by 2 to obtain the error between q1 and q2
'''
'''
def quat_error_rev(q_error, q1):
    if np.allclose(q_error, np.array([0, 0, 0, 0])):
        print("HERE")
        return q1

    return quatMultiply(q_error, q1)

    # q_error_conj = np.array([q_error[0], -q_error[1], -q_error[2], -q_error[3]])
    # q2 = quatMultiply(q1, q_error_conj)
    # return q2
'''
'''

# quat2atti
#
# Determine the attitude error from a quaternion error
# Format: [roll_error, pitch_error, yaw_error]
def quat2atti(q_error):
    """
    Computes attitude error from quaternion error

    Args:
    - q_error, 4-dimensional quaternion error [qs, qi, qj, qk]

    Returns:
    - atti_error: 3-dimensional attitude error [roll, pitch, yaw]
    """
    if np.allclose(q_error, np.array([0, 0, 0, 0])):
        return np.array([0, 0, 0])

    q_error /= np.linalg.norm(q_error)

    # Compute roll, pitch, and yaw from the quaternion
    sinr_cosp = 2.0 * (q_error[0] * q_error[1] + q_error[2] * q_error[3])
    cosr_cosp = 1.0 - 2.0 * (q_error[1] * q_error[1] + q_error[2] * q_error[2])
    roll_error = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q_error[0] * q_error[2] - q_error[3] * q_error[1])
    if np.abs(sinp) >= 1:
        pitch_error = np.sign(sinp) * np.pi / 2.0
    else:
        pitch_error = np.arcsin(sinp)

    siny_cosp = 2.0 * (q_error[0] * q_error[3] + q_error[1] * q_error[2])
    cosy_cosp = 1.0 - 2.0 * (q_error[2] * q_error[2] + q_error[3] * q_error[3])
    yaw_error = np.arctan2(siny_cosp, cosy_cosp)

    # Combine roll, pitch, and yaw into a numpy array
    atti_error = np.array([roll_error, pitch_error, yaw_error])

    return atti_error
'''
'''
# atti2quat
#
# Convert attitude error into a normalized quaternion error
def atti2quat(atti):
    """
    Computes quaternion error from attitude error

    Args:
    - atti_error: 3-dimensional attitude error [roll, pitch, yaw]

    Returns:
    - q_error, 4-dimensional quaternion error [qs, qi, qj, qk]
    """
    roll, pitch, yaw = atti

    # Compute half angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    # Combine into a numpy array and normalize
    q_error = np.array([qw, qx, qy, qz])
    q_error /= np.linalg.norm(q_error)

    return q_error
'''