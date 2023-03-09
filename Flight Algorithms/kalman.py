#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler

PURPOSE: Implements Extended Kalman Filtering for our Amateur Rocketry Flight Computer

OBJECTS: EKF(x, q, P, Q, R, f, F, h, H)

METHODS: EKF.predict(), EKF.update(z)

SEE BELOW FOR MORE DOCUMENTATION
    
"""

import numpy as np

import strapdown as sd
import quaternions as qt
import earth_model as em
import data_collection as dc


plot_vector = np.zeros((18000,1))
i = 0

class EKF:
    '''
    Extended Kalman Filter Implementation
    
    See __init__ for initialization
    
    See predict() and update() for information on running the filter
    '''
    
    def __init__(self, x, q, P, Q, R, f, F, h, H):
        '''
        Initializes the EKF object
        
        Arguments:
            - x: state vector, 9 x 9, [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll_error, pitch_error, yaw_error]
            - q: initial best estimate of quaternion, 4 x 1, 
            - P: predicted coviarnace matrix, 9 x 9, identity matrix
            - Q: measurement noise matrix, 9 x 9, identity matrix
            - R: 6 x 6 identitry matrix
            - f: see f() functon below
            - F: linearizeds state transition matrix, 9 x 9
            - h: see h() function below
            - H: see H() function below ... returns a 6 x 9 matrix
        '''
        
        self.x = x 
        self.q_global = q
        self.P = P
        self.Q = Q
        self.R = R
        self.f = f
        self.F = F
        self.h = h
        self.H = H 

    def predict(self):
        '''
        prediction step of Kalman filter - run this as frequently as possible
        
        Arguments:
            - none
            
        Returns:
            - none
            
        Notes:
            - Requires an initialized EKF object
        '''
        
        # predict state estimate
        self.x, self.q_global, self.F = self.f(self.x, self.q_global)
        
        # predict state covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # This is just for debugging purposes
        #print(plot_vector[i]) # why is this value so large on the 2nd iteration?
        global plot_vector, i
        plot_vector[i] = self.P[0,0]
        i += 1


    def update(self, z):
        '''
        prediction step of Kalman filter - run this when you have a new GPS or Barometer measurement
        Currently the global quaternion update from attitude error is not working
        
        Arguments:
            - z: measurement vector, 6x1
            
        Returns:
            - none
            
        Notes:
            - Requires an initialized EKF object
            
        Notes for Tyler: 
            - Currently, quaternion update via attitude error is not working, see commented sections below
        '''
        
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
        #print(self.x[6:9])
        
        # update state covariance (9 x 9)
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P
        
        '''
        Tyler: This is NOT working, but we think it should be
        Uncomment the first line #self.q_global = ... to see what we mean
        
        It's strange; the difference between q_global_new and self.q_global
        is small (on the order of 10e-14 to 10e-5) but apparently that's
        enough to make the results diverge? We think 10e-5 is too large
        
        It must be 10e-9 or less, see the second commented line
        '''
        
        # compute quaternion error from nonzero attitude error
        atti_error = self.x[6:9] # attitude error, nonzero because of update
        q_error = qt.atti2quat(atti_error) # quaternion error from attitude error
        
        # factor quaternion error into global quaternion
        q_global_new = qt.quatMultiply(q_error, self.q_global)
        #self.q_global = q_global_new # set global quaternion
        #self.q_global = self.q_global + np.ones((4,)) * 10e-9 # TESTING: error cannot exceed 10e-9
        
        self.x[6:9] = np.array([0, 0, 0])
        

def f(x, q_global):
    '''
    This function runs IMU strapdown, and should predict the attitude error from the quaternion
    
    Arguments:
        - x: state vector, 9 x 1, [pos_x, ... vel_x, ... roll_error, ...]
        - q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]

    Returns:
        - x_new: new state vector, 9 x 1
        - q_new: new quaternion estimate
        - F: updated state propagation matrix, 9 x 9, see F() functon
        
    Notes: for Tyler: Currently, attidude error prediction is not working, see commented sections below
    '''
    
    r_ecef, v_ecef, atti_error = x[0:3], x[3:6], x[6:9]
    
    # grab next IMU reading
    accel, gyro, dt = dc.get_next_imu_reading()
    dV_b_imu = accel * dt
    dTh_b_imu = gyro * dt

    
    # Run the IMU strapdown, get predictions including attitude (q_e2b_new)
    q_e2b = q_global
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt)
    
    #atti_error_new = np.array([0, 0, 0])
    q_global = q_e2b_new
    
    # Update state matrix
    x_new = np.concatenate((r_ecef_new, v_ecef_new, atti_error))

    return x_new, q_global, F(x, q_global, accel, gyro)


def h(x):
    '''
    This function converts xyz to lla to subtract from the measurement matrix
    Will eventually do this for barometer too, but we're not ready yet
    
    Arguments:
        x: state vector, 9 x 1
        
    Returns:
        a 6 x 1 vector of measurement prediction, [lat, lon, atti, baro1, baro2, baro3]
    '''
    
    # Unpack state variables (position)
    p = x[:3]

    # Convert position (xyz) to GPS coordinates (lla)
    gps_data = em.ecef2lla(p)

    return np.concatenate((gps_data, [0, 0, 0]))


def H(x):
    '''
    This function constructs the 6 x 9 H matrix
    Currently we have only constructed a smaller 3x3 matrix within the 6x9
    
    Arguments:
        x: state vector
        
    Returns:
        H: a 6 x 9 matrix
    '''
    
    # GPS to PVA
    r_to_lla = em.lla_jacobian(x[:3], HAE=True)
    v_to_lla = np.zeros((3,3)) # TODO: figure these out tonight
    a_to_lla = np.zeros((3,3)) # TODO: figure these out tonight
    
    # TODO: Barometer to PVA
    r_to_baro = np.zeros((3,3))
    v_to_baro = np.zeros((3,3))
    a_to_baro = np.zeros((3,3))
    
    H_mat = np.vstack([
        np.hstack([r_to_lla, v_to_lla, a_to_lla]),
        np.hstack([r_to_baro, v_to_baro, a_to_baro])
        ])
    
    return H_mat


def F(x, q, accel, gyro):
    '''
    This function constructs the 9 x 9 F matrix
    
    Arguments:
        x: state vector, 9 x 9, [pos_x, pos_y, pos_x, vel_x, ... ]
        q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]
        accel: IMU acceleration, 3 x 1, [accel_x, accel_y, accel_z], m/s^2
        gyro: IMU angular rotation, 3 x 1, [gyro_x, gyro_y, gyro_z], rad/sec
        
    Returns:
        F: a 9 x 9 matrix
    '''
    
    # unpack position
    r_ecef = x[:3]
    
    # determine rotation matrix and such
    T_b2i = np.linalg.inv(qt.quat2dcm(q))
    
    # determine cross of omega    
    omega_cross = skew(em.omega)

    # Compute each 3 x 3 submatrix ... Credit: Tyler's email
    drdr = np.zeros((3,3))
    drdv = np.eye(3)
    drdo = np.zeros((3,3))
    dvdr = em.grav_gradient(r_ecef) - np.square(omega_cross)
    dvdv = -2 * omega_cross
    dvdo = -T_b2i * skew(accel)
    dodr = np.zeros((3,3))
    dodv = np.zeros((3,3))
    dodo = -skew(gyro)

    # pack F matrix
    F = np.vstack([
            np.hstack([drdr, drdv, drdo]),
            np.hstack([dvdr, dvdv, dvdo]),
            np.hstack([dodr, dodv, dodo])
          ])
    
    return F


def skew(M):
    '''
    Computes the skew-symmetric matrix of a 3-element vector
    
    Arguments:
        - M: 3 x 1 vector
        
    Returns:
        - M x: 3 x 3 skew-symmetric matrix
    '''
    return np.cross(np.eye(3), M)


def initialize_ekf_state_vector():
    '''
    Initializes and returns the 9 x 9 EKF state matrix """
    
    Returns:
        - x: state vector, 9 x 9 [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll_error, pitch_error, yaw_error]
    '''
    
    # Position and velocity are initialized from the test file
    # TODO: read the test file instead of hard-coding these values
    vec = [1527850.153, -4464959.009, 4276353.59, 3.784561502,	1.295026731, -1.11E-16, 0, 0, 0]
    
    
    return np.array(vec)


def initialize_q_global():
    '''
    Initializes and returns the 4x1 global "truth" quaternion
    
    Returns:
        - q: initial quaternion estimate, 4 x 1, [qs, qi, qj, qk]
    '''
    
    # get GPS reading and convert to quaternion
    q_global = dc.get_first_quaternion()
    
    return q_global


def initialize_ekf_matrices(x, q):
    '''
    Initializes the P, Q, R, and F matrices
    
    Arguments:
        - x: state vector, 9 x 9, [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll_error, pitch_error, yaw_error]
        - q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]
        
    Returns:
        - P: 9 x 9
        - Q: 9 x 9
        - R: 6 x 6
        - F: 9 x 9 
    '''
    
    # P: predicted covariance matrix, 9 x 9, can be random (reflects initial uncertainty)
    P = np.eye((9)) * 0.1 # does not matter what this is

    # Q: measurement noise matrix, 9 x 9, I * 0.001
    Q = np.eye((9)) * 0.0001
    
    # R: 6 x 6, 
    # currently uncertain how to initialize
    R = np.diag([1, 1, 1, 0.01, 0.01, 0.01])
    
    # F: state propagation matrix, 9 x 9
    accel, gyro, dt = dc.get_next_imu_reading(advance=False)
    F_mat = F(x, q, accel, gyro) # we continuously update F so we have a function for it

    return P, Q, R, F_mat


'''
via ChatGPT
I'm pretty sure this can be accomplished with atti2quat, and quat_error_rev
'''

'''
def update_quaternion(atti_error, q_global):
    # Convert attitude error angles to rotation matrix
    roll_error, pitch_error, yaw_error = atti_error
    R_error_pitch = np.array([[np.cos(pitch_error), 0, np.sin(pitch_error)],
                              [0, 1, 0],
                              [-np.sin(pitch_error), 0, np.cos(pitch_error)]])
    
    R_error_roll = np.array([[1, 0, 0],
                             [0, np.cos(roll_error), -np.sin(roll_error)],
                             [0, np.sin(roll_error), np.cos(roll_error)]])
    
    R_error_yaw = np.array([[np.cos(yaw_error), -np.sin(yaw_error), 0],
                            [np.sin(yaw_error), np.cos(yaw_error), 0],
                            [0, 0, 1]])
    
    R_error = np.matmul(R_error_yaw, np.matmul(R_error_pitch, R_error_roll))

    # Convert quaternion to rotation matrix
    q_scalar, q_i, q_j, q_k = q_global
    R = np.array([[1 - 2*q_j**2 - 2*q_k**2, 2*q_i*q_j - 2*q_k*q_scalar, 2*q_i*q_k + 2*q_j*q_scalar],
                  [2*q_i*q_j + 2*q_k*q_scalar, 1 - 2*q_i**2 - 2*q_k**2, 2*q_j*q_k - 2*q_i*q_scalar],
                  [2*q_i*q_k - 2*q_j*q_scalar, 2*q_j*q_k + 2*q_i*q_scalar, 1 - 2*q_i**2 - 2*q_j**2]])

    # Multiply rotation matrices to get new rotation matrix
    R_new = np.matmul(R_error, R)

    # Convert new rotation matrix to quaternion
    q_scalar_new = 1/2 * np.sqrt(np.trace(R_new) + 1)
    q_i_new = (R_new[2, 1] - R_new[1, 2]) / (4 * q_scalar_new)
    q_j_new = (R_new[0, 2] - R_new[2, 0]) / (4 * q_scalar_new)
    q_k_new = (R_new[1, 0] - R_new[0, 1]) / (4 * q_scalar_new)

    # Normalize quaternion
    q_norm = np.sqrt(q_scalar_new**2 + q_i_new**2 + q_j_new**2 + q_k_new**2)
    q_scalar_new /= q_norm
    q_i_new /= q_norm
    q_j_new /= q_norm
    q_k_new /= q_norm

    # Return updated quaternion
    q_global_new = [q_scalar_new, q_i_new, q_j_new, q_k_new]
    return q_global_new
'''