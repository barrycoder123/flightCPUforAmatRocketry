# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 19:56:02 2023

@author: zrumm
"""

import numpy as np
import csv

class EKF:
    def __init__(self, x0, P0, Q, R):
        self.x = x0 # state estimate
        self.P = P0 # state covariance
        self.Q = Q # process noise covariance
        self.R = R # measurement noise covariance

    def predict(self, f, u, dt):
        # predict state estimate
        self.x = f(self.x, u, dt)
        # predict state covariance
        self.P = self.P + self.Q * dt

    def update(self, z, h):
        # compute innovation
        y = z - h(self.x)
        # compute innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        # compute Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # update state estimate
        self.x = self.x + K @ y
        # update state covariance
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P

def f_imu(x, u, dt):
    # IMU dynamics model
    # u = [ax, ay, az, gx, gy, gz]
    p = x[0:3]
    v = x[3:6]
    q = x[6:10]
    ax, ay, az = u[0:3]
    gx, gy, gz = u[3:6]
    # integrate velocity
    v = v + np.array([ax, ay, az]) * dt
    # integrate position
    p = p + v * dt
    # update attitude using quaternion kinematics
    omega = np.array([gx, gy, gz]) * dt
    dq = np.array([-0.5 * omega[0] * q[0] - 0.5 * omega[1] * q[1] - 0.5 * omega[2] * q[2],
                  0.5 * omega[0] * q[1] - 0.5 * omega[1] * q[0] - 0.5 * omega[2] * q[3],
                  0.5 * omega[0] * q[2] - 0.5 * omega[1] * q[3] - 0.5 * omega[2] * q[0],
                  0.5 * omega[0] * q[3] - 0.5 * omega[1] * q[2] - 0.5 * omega[2] * q[1]])
    q = q + dq * dt
    return np.concatenate((p, v, q))

def h_gps(x):
    # GPS measurement model
    return x[0:3]

def h_baro(x):
    # Barometer measurement model
    return x[2]



if __name__ == "__main__":
    # initialize state estimate, state covariance, process noise covariance, measurement noise covariance
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]) # [p, v, q]
    P0 = np.eye(10) * 100.0
    Q = np.eye(10) * 0.01
    R_gps = np.eye(3) * 0.1
    R_baro = np.array([0.1])

    # create an extended Kalman filter object
    ekf = EKF(x0, P0, Q, R)

    # input data (GPS, IMU, barometer)
    gps_data = np.loadtxt("gps_data.txt")
    imu_data = np.loadtxt("imu_data.txt")
    baro_data = np.loadtxt("baro_data.txt")

    # time step
    dt = 1.0

    # initialize time
    t = 0.0

    # initialize PVA
    pva = []

    # main loop
    for i in range(len(gps_data)):
        # predict
        ekf.predict(f_imu, imu_data[i, :], dt)

        # update with GPS
        if gps_data[i, 0] > 0.0:
            ekf.update(gps_data[i, 1:4], h_gps, R_gps)

        # update with barometer
        if baro_data[i, 0] > 0.0:
            ekf.update(baro_data[i, 1:2], h_baro, R_baro)

        # store PVA
        pva.append(np.concatenate((ekf.x[0:3], ekf.x[3:6], ekf.x[6:10])))

        # update time
        t = t + dt