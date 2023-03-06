#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler


PURPOSE: Library for Kalman filtering

CLASS: EKF

METHODS: EKF.predict(), EKF.update(z)
    
"""

import numpy as np

import strapdown as sd
import quaternions as qt
import earth_model as em
import data_collection as dc


class EKF:
    def __init__(self, x, global_quaternion, P, Q, R, f, F, h, H):
        
        self.x = x # state vector, 9x9
        self.global_quaternion = global_quaternion # global "truth" quaternion, 4x1
        self.P = P # predicted coviarnace matrix, 9x9
        self.Q = Q # measurement noise matrix, 9x9
        self.R = R # 
        self.f = f # IMU strapdown
        self.F = F # linearizeds state transition matrix
        self.h = h # GPS conversion
        self.H = H # jacobian of lla


    # predict step of Kalman filtering
    def predict(self):
        # predict state estimate
        self.x, self.global_quaternion = self.f(self.x, self.global_quaternion)
        
        # predict state covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        #self.P, self.Q, self.R, self.F = initialize_ekf_matrices(self.x, self.global_quaternion)


    # update step of Kalman filtering
    def update(self, z):
        # compute innovation
        y = z - self.h(self.x)
        
        # get H matrix (6 x 9)
        H = self.H(self.x[:3])
        
        # compute innovation covariance (6 x 6)
        S = H @ self.P @ H.T + self.R
        
        # compute Kalman gain (9 x 6)
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # update state estimate (9,)
        self.x = self.x + K @ y
        
        # update state covariance (9 x 9)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P
        
        # Tyler: apply attitude error to truth quaternion
        #self.global_quaternion = qt.atti2quat2(self.x[6:9], self.global_quaternion)
        #self.global_quaternion = qt.quatMultiply(q_err, self.global_quaternion)
    
        self.x[-3:] = [0, 0, 0] # reset attitude error (Tyler)
        


# IMU CONVERSION EQUATION
def f(x, global_quaternion):
    
    r_ecef, v_ecef, atti_error = x[0:3], x[3:6], x[6:9]
    
    # grab next IMU reading
    accel, gyro, dt = dc.get_next_imu_reading()
    dV_b_imu = accel * dt
    dTh_b_imu = gyro * dt
    
    # pass global quaternion into 
    q_e2b = global_quaternion
    
    # iterate a strapdown
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);
    
    
    atti_error_new = qt.quat2atti(global_quaternion, q_e2b_new)
    
    print(atti_error_new)
    
    """
    # F: state propagation matrix, 9x9 ... Credit: Tyler's Email
    omega_cross = skew(em.omega)
    r_ecef = x[:3]
    T_b2i = np.linalg.inv(qt.quat2dcm(global_quaternion))
    
    # Credit: Tyler's email
    drdr = np.zeros((3,3))
    drdv = np.eye(3)
    drdo = np.zeros((3,3))
    dvdr = em.grav_gradient(r_ecef) - np.square(omega_cross)
    dvdv = -2 * omega_cross
    dvdo = -T_b2i * skew(accel)
    dodr = np.zeros((3,3))
    dodv = np.zeros((3,3))
    dodo = -skew(gyro)

    F = np.vstack([
            np.hstack([drdr, drdv, drdo]),
            np.hstack([dvdr, dvdv, dvdo]),
            np.hstack([dodr, dodv, dodo])
          ])
    """
    
    
    return np.concatenate((r_ecef_new, v_ecef_new, atti_error_new)), q_e2b_new #, F


# GPS CONVERSION EQUATION
def h(x):
    # Unpack state variables (position)
    p = x[:3]

    # Convert position (xyz) to GPS coordinates (lla)
    gps_data = em.ecef2lla(p)

    return np.concatenate((gps_data, [0, 0, 0]))


# H 
#
# Construct the 6x9 H matrix
def H(x):

    # GPS
    r_to_lla = em.lla_jacobian(x[:3], HAE=True)
    v_to_lla = np.zeros((3,3)) # TODO: figure these out tonight
    a_to_lla = np.zeros((3,3)) # TODO: figure these out tonight
    
    # TODO: Add some barometer stuff later
    r_to_baro = np.zeros((3,3))
    v_to_baro = np.zeros((3,3))
    a_to_baro = np.zeros((3,3))
    
    
    H_mat = np.vstack([
        np.hstack([r_to_lla, v_to_lla, a_to_lla]),
        np.hstack([r_to_baro, v_to_baro, a_to_baro])
        ])
    
    return H_mat

# skew
#
# compute the skew matrix of a 3-element vector
def skew(M):
    return np.cross(np.eye(3), M)


# initializes the 9x9 EKF state matrix """
# state vector: [pos_x, 
#                pos_y, 
#                pos_z, 
#                vel_x, 
#                vel_y, 
#                vel_z, 
#                roll_error, 
#                pitch_error, 
#                yaw_error]
def initialize_ekf_state_vector():
    
    # get current GPS reading and convert to ECEF
    gps_data, dt = dc.get_next_gps_reading(advance=False) 
    x, y, z = em.lla2ecef(gps_data) 
    vec = [1527850.153, -4464959.009, 4276353.59, 3.784561502,	1.295026731, -1.11E-16, 0, 0, 0]
    
    # velocity and attitude error are initially 0
    return np.array(vec)
    #return np.array([x, y, z, 0, 0, 0, 0, 0, 0])


# initializes the 4x1 global "truth" quaternion
def initialize_global_quaternion():
    
    # get GPS reading and convert to quaternion
    global_quaternion = dc.get_first_quaternion()
    
    return global_quaternion


# initializes the P, Q, R, and F matrices
# TODO: probably don't want to initialize these with identity matrices
def initialize_ekf_matrices(x, global_quaternion):
    
    # P: predicted covariance matrix (9 x 9), can be random (reflects initial uncertainty)
    P = np.eye((9)) * 0.1 # does not matter what this is

    # Q: measurement noise matrix, 10x10, I * 0.001
    # Initialize the Q matrix to reflect the uncertainty in the system dynamics
    #Q = np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
    Q = np.eye((9)) * 0.01
    
    # R: 6x6, uncertain how to initialize
    # Initialize the R matrix to reflect the uncertainty in the sensor measurements
    # GPS has noise of 2.5 meters ?
    R = np.diag([2.5, 2.5, .25, 0.01, 0.01, 0.01])
    #R = np.eye((6)) * 2.5
    
    # F: state propagation matrix, 9x9 ... Credit: Tyler's Email
    omega_cross = skew(em.omega)
    r_ecef = x[:3]
    T_b2i = np.linalg.inv(qt.quat2dcm(global_quaternion))
    
    # Read IMU to determine accel and angular gyro
    accel, gyro, dt = dc.get_next_imu_reading(advance=False)
    
    # Credit: Tyler's email
    drdr = np.zeros((3,3))
    drdv = np.eye(3)
    drdo = np.zeros((3,3))
    dvdr = em.grav_gradient(r_ecef) - np.square(omega_cross)
    dvdv = -2 * omega_cross
    dvdo = -T_b2i * skew(accel)
    dodr = np.zeros((3,3))
    dodv = np.zeros((3,3))
    dodo = -skew(gyro)

    F = np.vstack([
            np.hstack([drdr, drdv, drdo]),
            np.hstack([dvdr, dvdv, dvdo]),
            np.hstack([dodr, dodv, dodo])
          ])
    
    return P, Q, R, F       