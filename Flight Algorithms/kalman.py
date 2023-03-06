#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler


PURPOSE: Library for Kalman filtering


FCLASS: EKF

METHODS: EKF.predict(), EKF.update(z)
    
"""

import numpy as np

import strapdown as sd
import quaternions as qt
import earth_model as em
import data_collection as dc


class EKF:
    def __init__(self, x, q_true, P, Q, R, f, F, h, H):
        
        self.x = x # state vector, 9x9
        self.q_true = q_true # global "truth" quaternion, 4x1
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
        self.x, dt = self.f(self.x, self.q_true)
        
        # predict state covariance
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        # compute innovation
        y = z - self.h(self.x)
        
        # get H matrix (6 x 9)
        H = self.H(x[:3])
        
        # compute innovation covariance (6 x 6)
        S = H @ self.P @ H.T + self.R
        
        # compute Kalman gain (9 x 6)
        K = self.P @ H.T @ S # np.linalg.inv(S)
        
        # update state estimate (9,)
        self.x = self.x + K @ y
        
        # update state covariance (9 x 9)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P
        
        # Tyler: apply attitude error to truth quaternion
        self.q_true = qt.atti2quat(self.x[-3:], self.q_true)
        self.x[-3:] = [0, 0, 0] # reset error (Tyler)


# IMU CONVERSION EQUATION
def f(x, q_true):
    
    r_ecef, v_ecef, atti_error = x[0:3], x[3:6], x[6:9]
    
    # grab next IMU reading
    accel, gyro, dt = dc.get_next_imu_reading()
    dV_b_imu = accel * dt
    dTh_b_imu = gyro * dt
    
    q_e2b = qt.atti2quat(atti_error, q_true)
    
    # iterate a strapdown
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);

    # convert quaternion to attitude error
    atti_error_new = qt.quat2atti(q_e2b_new, q_true)

    return np.concatenate((r_ecef_new, v_ecef_new, atti_error_new)), dt


# GPS CONVERSION EQUATION
def h(x):
    # Unpack state variables (position)
    p = x[:3]

    # Convert position (xyz) to GPS coordinates (lla)
    gps_data = em.ecef2lla(p)

    return np.concatenate((gps_data, [0, 0, 0]))


# H 
#
# Should be 6x9
def H(x):
    # TODO: May need to add some barometer stuff
    return np.zeros((6,9))
    #return em.lla_jacobian(x[:3], HAE=True)

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
    
    # get GPS reading and convert to ECEF
    gps_data, dt = dc.get_next_gps_reading(advance=False) 
    x, y, z = em.lla2ecef(gps_data) 
    
    # velocity and attitude error are initially 0
    return np.array([x, y, z, 0, 0, 0, 0, 0, 0])

# initializes the 4x1 global quaternion
def initialize_global_quaternion():
    
    # get GPS reading and convert to quaternion
    q_true = dc.get_first_quaternion()
    
    return q_true

# initializes the P, Q, R, and F matrices
def initialize_ekf_matrices():
    
    omega_cross = np.vstack([[0, -em.omega[2], em.omega[1]],
                   [em.omega[2], 0, -em.omega[0]],
                   [-em.omega[1], em.omega[0], 0]])
    am_cross = omega_cross
    
    
    x = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    
    # measurement vector: [lat, lon, atti, baro1, baro2, baro3]
    z = np.array([0, 0, 0, 0, 0, 0])

    # Initialize P, Q, R
    # P: predicted covariance matrix, can be random, 9x9, maybe I * .1
    # Q: measurement noise matrix, 10x10, I * 0.001
    # R: 6x6
    # F: 10x10, given by Tyler (add a row of 0's)
    P = np.zeros((9,9))
    Q = np.zeros((9,9))
    R = np.zeros((6,6))
    F = np.zeros((9,9))
    

    # TODO: DEFINE THE F MATRIX
    grav_gradient = np.zeros((3,3))
    ang_rot_cross = np.zeros((3,3))
    T_b2i = np.zeros((3,3))
    
    drdr = np.zeros((3,3))
    drdv = np.identity(3)
    drdo = np.zeros((3,3))
    dvdr = grav_gradient - np.square(omega_cross)
    dvdv = -2 * omega_cross
    dvdo = -T_b2i * am_cross
    dodr = np.zeros((3,3))
    dodv = np.zeros((3,3))
    dodo = -ang_rot_cross

    F = np.vstack([
            np.hstack([drdr, drdv, drdo]),
            np.hstack([dvdr, dvdv, dvdo]),
            np.hstack([dodr, dodv, dodo])
          ])
    
    return P, Q, R, F
    

if __name__ == "__main__":
    
    # initialize state vectors and matrices
    dc.reset()
    x = initialize_ekf_state_vector()
    q_true = initialize_global_quaternion()
    P, Q, R, F = initialize_ekf_matrices()
    
    # Initialize EKF
    ekf = EKF(x, q_true, P, Q, R, f, F, h, H)

    # EKF loop
    print("RUNNING EKF")
    while 1:
        if dc.done():
            break
        
        # Predict state
        ekf.predict()
        
        # If new GPS reading, update state
        if dc.gps_is_ready():
            lla, dt = dc.get_next_gps_reading()
            baro = dc.get_next_barometer_reading() # TODO: implement
            
            z = np.concatenate((lla, baro))
            ekf.update(z) 
            
        
        
    print("DONE WITH KALMAN FILTERING")
            
            
            