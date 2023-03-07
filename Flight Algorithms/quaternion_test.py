#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 15:02:23 2023

@author: zrummler
"""

import numpy as np
import quaternions as qt


def attitude_error(q_true, q_predict):
    """Compute the attitude error between two quaternions.

    Args:
        q_true (np.ndarray): The true attitude quaternion as a 4D array.
        q_predict (np.ndarray): The predicted attitude quaternion as a 4D array.

    Returns:
        np.ndarray: The attitude error as a 3-element vector in radians.
    """
    q_error = qt.quatMultiply(q_predict, qt.quat_inv(q_true))
    roll_error = np.arctan2(2 * (q_error[0] * q_error[1] - q_error[2] * q_error[3]),
                            1 - 2 * (q_error[1] ** 2 + q_error[2] ** 2))
    pitch_error = np.arcsin(2 * (q_error[0] * q_error[2] - q_error[3] * q_error[1]))
    yaw_error = np.arctan2(2 * (q_error[0] * q_error[3] + q_error[1] * q_error[2]),
                           1 - 2 * (q_error[2] ** 2 + q_error[3] ** 2))
    return np.array([roll_error, pitch_error, yaw_error])


def apply_attitude_error(q_true, attitude_error):
    """Apply the attitude error to a quaternion.

    Args:
        q_true (np.ndarray): The true attitude quaternion as a 4D array.
        attitude_error (np.ndarray): The attitude error as a 3-element vector in radians.

    Returns:
        np.ndarray: The updated quaternion as a 4D array.
    """
    roll_error, pitch_error, yaw_error = attitude_error
    c1, c2, c3 = np.cos(roll_error/2), np.cos(pitch_error/2), np.cos(yaw_error/2)
    s1, s2, s3 = np.sin(roll_error/2), np.sin(pitch_error/2), np.sin(yaw_error/2)
    q_error = np.array([c1*c2*c3 + s1*s2*s3,
                        s1*c2*c3 - c1*s2*s3,
                        c1*s2*c3 + s1*c2*s3,
                        c1*c2*s3 - s1*s2*c3])
    q_new = np.quaternion(*q_error) * np.quaternion(*q_true)
    return np.array([q_new[1], q_new[2], q_new[3], q_new[0]])


# Unit tests for attitude_error and apply_attitude_error
def test_attitude_error():
    # Test case 1
    q_true = np.array([0.0, 0.0, 0.0, 1.0])
    q_predict = np.array([1.0, 0.0, 0.0, 0.0])
    error = attitude_error(q_true, q_predict)
    print(error)
    assert np.allclose(error, np.array([np.pi/2, 0, 0]))

    # Test case 2
    q_true = np.array([1.0, 0.0, 0.0, 0.0])
    q_predict = np.array([0.0, 1.0, 0.0, 0.0])
    error = attitude_error(q_true, q_predict)
    assert np.allclose(error, np.array([np.pi, 0, 0]))

    # Test case 3
    #q_true = np


test_attitude_error()