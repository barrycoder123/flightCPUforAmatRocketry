import numpy as np

import strapdown as sd
import quaternions as qt
import earth_model as em
import data_collection as dc


class EKF:
    def __init__(self, x, q_true, P, Q, R, f, F, h, H):
        
        self.x = x # state vector, 9x9
        self.q_true # global "truth" quaternion, 4x1
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
        # compute innovation covariance
        H = self.H(x[:3]) # compute H matrix
        S = H @ self.P @ H.transpose() + self.R
        # compute Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # update state estimate
        self.x = self.x + K @ y
        # update state covariance
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P


# IMU CONVERSION EQUATION
def f(x, q_true):
    
    r_ecef, v_ecef, atti_error = x[0:3], x[3:6], x[6:10]
    
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
    lat, lon, alt = em.ecef2lla(p[0], p[1], p[2])

    # TODO: Placeholders for when we add barometer
    baro1 = 0
    baro2 = 0
    baro3 = 0 

    return np.array([lat, lon, alt, baro1, baro2, baro3])


# H 
def H(x):
    # TODO: May need to add some barometer stuff
    return em.lla_jacobian(x[:3], HAE=True)

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
    x, y, z = em.lla2ecef(gps_data[0], gps_data[1], gps_data[2]) 
    
    # velocity and attitude error are initially 0
    return np.array([x, y, z, 0, 0, 0, 0, 0, 0])

# initializes the 4x1 global quaternion
def initialize_global_quaternion():
    
    # get GPS reading and convert to quaternion
    q_true = dc.get_first_quaternion()
    print(q_true)
    
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
    # P: predicted covariance matrix, can be random, 10x10, maybe I * .1
    # Q: measurement noise matrix, 10x10, I * 0.001
    # R: 
    # F: 10x10, given by Tyler (add a row of 0's)
    P = np.zeros((10,10))
    Q = np.zeros((10,10))
    R = np.zeros((10,10))
    F = np.zeros((10,10))
    

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
    x = initialize_ekf_state_vector()
    q_true = initialize_global_quaternion()
    P, Q, R, F = initialize_ekf_matrices()
    
    # Initialize EKF
    ekf = EKF(x, q_true, P, Q, R, f, F, h, H)

    # EKF loop
    while 1:
        # Predict state
        ekf.predict()
        
        # If new GPS reading, update state
        if dc.gps_is_ready():
            lat, long, atti, dt = dc.get_next_gps_reading()
            baro1, baro2, baro3 = dc.get_next_barometer_reading() # TODO: implement
            
            z = [lat, long, atti, baro1, baro2, baro3]
            ekf.update(z) 
            
            # TODO: apply atti error to true quaternion
            # TODO: reset error to 0
            
            
            