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

np.set_printoptions(linewidth=200)

import strapdown as sd
import quaternions as qt
import earth_model as em
import data_collection as dc


class EKF:
    """
    Extended Kalman Filter Implementation

    See __init__ for initialization

    See predict() and update() for information on running the filter
    """

    def __init__(self, x0, q0_e2b, P0, Q):
        """
        Initializes the EKF object

        Arguments:
            - x0: (9,1) or (9,) initial state vector [r_ecef, v_ecef, roll_error, pitch_error, yaw_error]
            - q0: initial best estimate of quaternion, 4 x 1,
            - P0: (9,9) initial covariance matrix, 9 x 9, identity matrix
            - Q: (9,9) process noise covariance matrix
            - R: 6 x 6 identitry matrix
            - F: linearizeds state transition matrix, 9 x 9
            - h: see h() function below
            - H: see H() function below ... returns a 6 x 9 matrix
        """

        self.x = x0.flatten()
        self.q_e2b = q0_e2b
        self.P = P0
        self.Q = Q

    def predict(self):
        """
        prediction step of Kalman filter - run this as frequently as possible
        
        Arguments:
            - none
            
        Returns:
            - none
            
        Notes:
            - Requires an initialized EKF object
        """

        # predict state estimate
        self.x, self.q_e2b, phi = f(self.x.flatten(), self.q_e2b)

        # predict state covariance
        self.P = phi @ self.P @ phi.T + self.Q

    def update(self, z_gps, z_baro=None, sigma_gps=15, sigma_baro=0.1):
        """
        EKF measurement update
        prediction step of Kalman filter - run this when you have a new GPS or Barometer measurement
        Currently the global quaternion update from attitude error is not working

        Arguments:
            - z: measurement vector, 6x1

        Returns:
            - none

        Notes:
            - Requires an initialized EKF object
        """

        # get the measurements
        z_gps_ecef = em.lla2ecef(z_gps)
        nu, H, R = get_position_measurement(self.x, z_gps_ecef, sigma_gps)  # GPS measurement
        if z_baro is not None:
            # todo: compute nu, H, R for barometer
            # todo: use vstack and blockdiag to combine nu, H, and R as needed
            raise NotImplementedError('Barometer measurement not yet implemented')

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


def f(x, q_e2b):
    """
    This function runs IMU strapdown, and should predict the attitude error from the quaternion

    Arguments:
        - x: state vector, 9 x 1, [pos_x, ... vel_x, ... roll_error, ...]
        - q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]

    Returns:
        - x_new: new state vector, 9 x 1
        - q_new: new quaternion estimate
        - F: updated state propagation matrix, 9 x 9, see F() functon

    Notes: for Tyler: Currently, attidude error prediction is not working, see commented sections below
    """

    r_ecef, v_ecef = x[0:3], x[3:6]  # extract ECEF states for convenience

    # grab next IMU reading
    accel, gyro, dt = dc.get_next_imu_reading()
    dV_b_imu = accel * dt
    dTh_b_imu = gyro * dt

    # Run the IMU strapdown, get predictions including attitude (q_e2b_new)
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt)

    # Update state matrix
    x_new = np.concatenate((r_ecef_new, v_ecef_new, np.zeros(r_ecef.shape)))

    # compute linearized state transition matrix
    phi = compute_state_transition_matrix(dt, x, q_e2b, accel, gyro)
    return x_new, q_e2b_new, phi


def get_altitude_measurement(x, alt_meas: float, sigma: float = 5.0):
    """
    Gets an altitude measurement and the accompanying measurement Jacobian. The altitude is expected to be measure in Height Above the Ellipsoid (HAE) which
    may not be the most useful coordinate frame. This was not used in the software and thus was never modified.

    Parameters
    ----------
    x : (N,) ndarray
        state vector

    alt_meas : float
        measured altitude in HAE [m]

    sigma : float
        measurement standard deviation [m] (Default: 5)

    Returns
    -------
    nu : float
        measurement innovation vector

    H : (1,N) ndarray
        measurement partial matrix

    R : float
        measurement variance

    """

    lla = em.ecef2lla(x[0:3])  # convert to LLA in [rad, rad, m (HAE)]
    H = np.zeros((1, x.shape[0]))  # measurement partial
    H[0, 0] = np.cos(lla[1]) * np.cos(lla[0])  # partials calculated using ECEF to LLA function; not numerically validated
    H[0, 1] = np.sin(lla[1]) * np.cos(lla[0])
    H[0, 2] = np.sin(lla[0])
    nu = alt_meas - lla[2]
    if nu.ndim < 2:
        nu = np.expand_dims(nu, axis=1)
    R = sigma ** 2
    return nu, H, R


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


def initialize_ekf_state_vector():
    """
    Initializes and returns the 9 x 9 EKF state matrix

    Returns:
        - x: state vector, 9 x 9 [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll_error, pitch_error, yaw_error]
    """

    # Position and velocity are initialized from the test file
    # TODO: read the test file instead of hard-coding these values
    vec = [1527850.153, -4464959.009, 4276353.59, 3.784561502, 1.295026731, -1.11E-16, 0, 0, 0]

    return np.array(vec)


def initialize_q_global():
    """
    Initializes and returns the 4x1 global "truth" quaternion

    Returns:
        - q: initial quaternion estimate, 4 x 1, [qs, qi, qj, qk]
    """

    # get GPS reading and convert to quaternion
    q_e2b = dc.get_first_quaternion()

    return q_e2b


def initialize_ekf_matrices(x, q):
    """
    Initializes the P, Q, R, and F matrices

    Arguments:
        - x: state vector, 9 x 9, [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll_error, pitch_error, yaw_error]
        - q: best quaternion estimate, 4 x 1, [qs, qi, qj, qk]

    Returns:
        - P: 9 x 9
        - Q: 9 x 9
        - R: 6 x 6
        - F: 9 x 9
    """

    # P: predicted covariance matrix, 9 x 9, can be random (reflects initial uncertainty)
    P = np.eye(9) * 0.1  # does not matter what this is

    # Q: process noise matrix, 9 x 9, I * 0.001
    Q = np.eye(9) * 0.001
    return P, Q
