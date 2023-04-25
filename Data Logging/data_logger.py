"""
Created on Thu Mar 16 15:16:51 2023

@author: zrummler

If num_points is None, then 

"""

import time
import datetime
import numpy as np
#import matplotlib.pyplot as plt

colnames = [
    "t [sec]",
    "r_ecef_X",
    "r_ecef_Y",
    "r_ecef_Z",
    "v_ecef_X",
    "v_ecef_Y",
    "v_ecef_Z",
    "q_e2b_scalar",
    "q_e2b_i",
    "q_e2b_j",
    "q_e2b_k",
    "accel_X [m/s^2]",
    "accel_Y [m/s^2]",
    "accel_Z [m/s^2]",
    "gyro_X [rad/s]",
    "gyro_Y [rad/s]",
    "gyro_Z [rad/s]",
    "gps_lat [deg]",
    "gps_long [deg]",
    "gps_alt [m]"]

class DataLogger:
        
    def __init__(self, initial_state_vector, initial_quaternion, num_points=None):
        """
        initializes the DataLogger class
        
        Arguments:
            initial_state_vector: array (10,1) of the form [pos, vel, atti_error]
            initial_quaternion: array (4,1)
            num_points: int (optional) number of rows of the data file
        """
        width = len(colnames)
        avg_dt = 0.0876 # experimentally determined
        
        if num_points is not None:   
            self.PVA_est = np.zeros((width, num_points+1)) # save room for columns
            
        else:
            big_number = 2**14 # at dt=0.01, this is 11 minutes of data, should be fine 
            print("will record for",big_number * avg_dt / 60,"minutes")
            self.PVA_est = np.zeros((width,big_number+1))
        
        
        # write initial conditions
        self.PVA_est[1:7, 0] = initial_state_vector[:6]
        self.PVA_est[7:11, 0] = initial_quaternion
        
        # first position to write to
        self.write_pos = 1
           
        
    def start_timer(self):
        """
        Starts the timer. Must call this right before you start saving data
        
        No arguments, no return value
        """
        
        self.t_initial = time.perf_counter()
        self.PVA_est[0, 1] = 0
            
    def save_state_to_buffer(self, state_vector, quaternion, imu_reading, gps_reading):
        """
        Saves one line to a buffer
        
        Arguments:
            state_vector: array (10,1), pos_xyz, vel_xyz, atti_xyz
            quaternion: array (4,1)
            imu_reading: array (6,1), accel_xyz, gyro_xyz
            gps_reading: array (3,1), gps_lla
            
        Returns:
            none
        """
        
        self.PVA_est[0, self.write_pos] = time.perf_counter() - self.t_initial
        self.PVA_est[1:7, self.write_pos] = state_vector[:6].flatten()  # store position and velocity only
        self.PVA_est[7:11, self.write_pos] = quaternion.flatten() # store the quaternion
        self.PVA_est[11:17, self.write_pos] = imu_reading
        self.PVA_est[17:, self.write_pos] = gps_reading
    
        self.write_pos += 1     
        
        
    def write_buffer_to_file(self):
        """
        Writes the buffer to a .csv file. Must call this after you finish saving data
        
        Returns:
            filename: (str) string of the file
        """
        
        # create a formatted filename string with the date and time
        now = datetime.datetime.now()
        filename = now.strftime("data_%Y-%m-%d_%H-%M-%S.csv")
        
        # truncate array
        self.PVA_est = self.PVA_est[:,:self.write_pos]
        
        # write the data to the file
        col_str = ','.join(colnames)
        np.savetxt(filename, self.PVA_est.T, delimiter=',', fmt='%s', header=col_str)
        
        return filename
        
    
    def print_position_drift(self):
        """
        Plots how much the data drifts by. Helpful for debugging
        """
        
        num_points = self.PVA_est.shape[1]
        
        print("POSITION DRIFT:")
        
        for i in range(num_points):
            curr_time = self.PVA_est[0,i]
            drift_xyz = self.PVA_est[1:4,i] - self.PVA_est[1:4,0]
            
            print(curr_time, drift_xyz)