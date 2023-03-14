#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 12:22:46 2023

@author: zrummler

PURPOSE: Implements Extended Kalman Filtering for our Flight Computer

OBJECT: EKF(x, q, P, Q, R, f, F, h, H)

METHODS: EKF.predict(), EKF.update(z)

SEE BELOW FOR MORE DOCUMENTATION
    
"""

import scipy
import numpy as np

np.set_printoptions(linewidth=200)

import strapdown as sd
import quaternions as qt
import earth_model as em


class EKF:
    """
    Extended Kalman Filter Implementation

    How to initialize:
        - Initialize 9-element state vector, x
        - Initialize 4-element global quaternion, q_e2b
        - Create a class with ekf = EKF(x, q_e2b)

    How to run:
        
        while data collection
            accel, gyro, dt = get next IMU reading
            ekf.predict(accel, gyro, dt)
            
            if gps or barometer ready
                lla = get next GPS data
                ekf.update(lla)

    See predict() and update() for information on running the filter
    """

    def __init__(self, x0, q0_e2b):
        """
        Initializes the EKF object

        Arguments:
            - x0: (9,1) or (9,) initial state vector [r_ecef, v_ecef, roll_error, pitch_error, yaw_error]
            - q0: initial best estimate of quaternion, 4 x 1,
        """

        self.x = x0.flatten()
        self.q_e2b = q0_e2b
        self.P, self.Q = init_ekf_matrices(x0, q0_e2b)

    def predict(self, z_imu, dt):
        """
        EKF state prediction - run this when you have a new IMU reading
        
        Arguments:
            - z_imu: (6,1) or (6,) IMU reading [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
            - dt: time step since last reading, seconds
            
        Returns:
            - None
            
        Notes:
            - Requires an initialized EKF object
        """

        # predict state estimate
        self.x, self.q_e2b, phi = f(self.x.flatten(), self.q_e2b, z_imu, dt)

        # predict state covariance
        self.P = phi @ self.P @ phi.T + self.Q

    def update(self, z_gps, z_baro=None, sigma_gps=15, sigma_baro=0.1):
        """
        EKF measurement update - run this when you have a new GPS or Barometer measurement

        Arguments:
            - z_gps: (3,1) or (3,) gps measurement vector [lat, long, alt]
            - z_baro: (3,1) or (3,) barometer measurement vector [alt1, alt2, alt3]
            - sigma_gps: standard deviation of GPS readings
            - sigma_baro: standard deviation of barmometer readings

        Returns:
            - None

        Notes:
            - Requires an initialized EKF object
        """

        # get the measurements
        z_gps_ecef = em.lla2ecef(z_gps)
        nu, H, R = get_position_measurement(self.x, z_gps_ecef, sigma_gps)  # GPS measurement    
            
        if z_baro is not None:
            
            # compute nu, H, R for barometer
            nu_baro, H_baro, R_baro = get_altitude_measurement(self.x, z_baro, sigma_baro)
    
            # use vstack and blockdiag to combine nu, H, and R as needed
            nu = np.vstack((nu, nu_baro))
            H = np.vstack((H, H_baro))
            R = scipy.linalg.block_diag(R, R_baro)
            
            #raise NotImplementedError('Barometer measurement not yet implemented')

        # generic EKF update equations
        S = H @ self.P @ H.T + R  # innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x.reshape(-1, 1) + K @ nu  # update state vector
        IKH = np.eye(self.x.shape[0]) - K.dot(H)  # intermediate variable
        self.P = IKH.dot(self.P).dot(IKH.T) + K.dot(R).dot(K.T)  # update state covariance (9 x 9)
        self.x = self.x.flatten()  # ensure x is 1D

        # Reset the attitude state.  Move attitude correction from x to q
        q_error = qt.deltaAngleToDeltaQuat(-self.x[6:9])
        self.q_e2b = qt.quatMultiply(q_error, self.q_e2b).flatten()
        self.x[6:9] = 0  # reset attitude error


def f(x, q_e2b, z_imu, dt):
    """
    This function updates the state vector and global quaternion via IMU strapdown. 
    It also updates the state transiction matrix (9 x 9)

    Arguments:
        - x: (9,1) or (9,) state vector, [pos_x, ... vel_x, ... roll_error, ...]
        - q_eqb: (4,1) or (4,) global quaternion [q_scalar, qi, qj, qk]

    Returns:
        - x_new: updated state vector
        - q_new: updated global quaternion 
        - phi: (9,9) updated state propagation matrix
    """

    r_ecef, v_ecef = x[0:3], x[3:6]  # extract ECEF states for convenience

    # grab next IMU reading
    accel, gyro = z_imu[0:3], z_imu[3:6]
    dV_b_imu = accel * dt
    dTh_b_imu = gyro * dt

    # Run the IMU strapdown, get predictions including attitude (q_e2b_new)
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt)

    # Update state matrix
    x_new = np.concatenate((r_ecef_new, v_ecef_new, np.zeros(r_ecef.shape)))

    # compute linearized state transition matrix
    phi = compute_state_transition_matrix(dt, x, q_e2b, accel, gyro)
    return x_new, q_e2b_new, phi


# Credit: Tyler Klein
def get_altitude_measurement(x, alt_meas: np.ndarray, sigma: float = 5.0):
    """
    Gets an altitude measurement and the accompanying measurement Jacobian. The altitude is expected to be measure in Height Above the Ellipsoid (HAE) which
    may not be the most useful coordinate frame. This was not used in the software and thus was never modified.

    Parameters
    ----------
    x : (N,) ndarray
        state vector

    alt_meas : (3,)
        measured altitude in HAE [m]

    sigma : float
        measurement standard deviation [m] (Default: 5)

    Returns
    -------
    nu : (3,1)
        measurement innovation vector

    H : (3,N) ndarray
        measurement partial matrix

    R : (3,3)
        measurement variance

    """

    lla = em.ecef2lla(x[0:3])  # convert to LLA in [rad, rad, m (HAE)]
    H = np.zeros((3, x.shape[0]))  # measurement partial
    H[0, 0] = np.cos(lla[1]) * np.cos(lla[0]) # partial derivative of alt_meas with respect to x
    H[0, 1] = np.sin(lla[1]) * np.cos(lla[0]) # partial derivative of alt_meas with respect to y
    H[0, 2] = np.sin(lla[0]) # partial derivative of alt_meas with respect to z
    
    nu = (alt_meas - lla[2]).reshape(3,1)

    R = sigma ** 2 * np.eye(3)
    return nu, H, R

# Credit: Tyler Klein
def get_position_measurement(x, z, sigma=15):
    """
    Gets an absolute position measurement in the ECEF frame

    Parameters
    ----------
    x : (N,) or (N,1) ndarray
        state vector where x[0:3] is the ECEF position in [meters]

    z : (3,) ndarray,
        measured ECEF position [meters]

    sigma : float, default=15
        measurement uncertainty [m] (Default: 15)

    Returns
    -------
    nu : (3,1) ndarray
        measurement innovation vector [meters]

    H : (3,N) ndarray
        measurement partial matrix

    R : (3,3) ndarray
        measurement covariance matrix

    """

    if np.ndim(x) == 2:
        x = x[:, 0]  # reduce dimension

    nu = (z - x[:3]).reshape(3, 1)  # measurement innovation
    R = sigma * sigma * np.eye(3)  # measurement covariance matrix
    H = np.zeros((3, x.shape[0]))
    H[:3, :3] = np.eye(3)
    return nu, H, R


def compute_state_transition_matrix(dt, x, q, accel, gyro):
    """
    This function constructs the 9 x 9 state transition matrix

    Arguments:
        dt: timestep [seconds]
        x: state vector, 9 x 9, [pos_x, pos_y, pos_x, vel_x, ... ]
        q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]
        accel: IMU acceleration, 3 x 1, [accel_x, accel_y, accel_z], m/s^2
        gyro: IMU angular rotation, 3 x 1, [gyro_x, gyro_y, gyro_z], rad/sec

    Returns:
        F: a 9 x 9 matrix
    """

    F = np.zeros((9, 9))

    # unpack position
    r_ecef = x[:3]

    # determine rotation matrix and such
    T_b2i = np.linalg.inv(qt.quat2dcm(q))

    # determine cross of omega    
    omega_cross = skew(em.omega)

    # Compute each 3 x 3 submatrix ... Credit: Tyler's email
    F[0:3, 3:6] = np.eye(3)  # drdv
    F[3:6, 0:3] = em.grav_gradient(r_ecef) - omega_cross.dot(omega_cross)  # dvdr
    F[3:6, 3:6] = -2 * omega_cross  # dvdv
    F[3:6, 6:9] = -T_b2i.dot(skew(accel))  # dvdo
    F[6:9, 6:9] = -skew(gyro)  # dodo
    F = np.eye(9) + F * dt
    return F


def skew(M):
    """
    Computes the skew-symmetric matrix of a 3-element vector

    Arguments:
        - M: 3 x 1 vector

    Returns:
        - M x: 3 x 3 skew-symmetric matrix
    """
    return np.cross(np.eye(3), M)


def init_ekf_matrices(x, q):
    """
    Initializes the P, Q, R, and F matrices

    Arguments:
        - x: state vector, 9 x 9, [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll_error, pitch_error, yaw_error]
        - q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]

    Returns:
        - P: 9 x 9
        - Q: 9 x 9
    """

    # P: predicted covariance matrix, 9 x 9, can be random (reflects initial uncertainty)
    P = np.eye(9) * 0.1  # does not matter what this is

    # Q: process noise matrix, 9 x 9, I * 0.001
    Q = np.eye(9) * 0.001
    return P, Q
