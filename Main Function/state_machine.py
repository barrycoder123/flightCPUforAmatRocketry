#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 27 11:32:11 2023

@author: zrummler

FlightCPUStateMachine class
"""

import sys
import time
import numpy as np

sys.path.append('../Data Collection')
sys.path.append('../Data Logging')
sys.path.append('../Flight Algorithms')

import kalman as kf
import data_logger as dl
import sensor_data_collector as dc

class StateMachine:
    
    def launch_detect(self):
        """
        Launch Detect State
        - Waits for sensors to warm up and subsequently initializes state vector and quaternion.
        """
        
        # 1: Initialize GPS, IMU, and Barometer objects. At this step the barometer should read the sea level barometric pressure from the config file.
        self.data_collector = dc.DataCollector()
        
        # 2: Wait 30 seconds. Allows GPS and IMU to warm up.
        wait_time = 0.1 # make this 30
        time.sleep(wait_time)
        
        # 3: Continually read from the GPS until a reading with at least 8 satellites is made. Once a reading with 8 satellites is made, save that reading [Lat, Lon, Alt]
        lla = self.data_collector.get_next_gps_reading()
        
        # 4: Initialize attitude with GPS and IMU reading. Currently: get_initial_state_and_quaternion() in Data collection. Need to re-write to take in LLA from valid GPS update (8+ satellites, and freshly calibrated and warmed up IMU).
        x, q_true = self.data_collector.get_initial_state_and_quaternion(lla)
        self.ekf = kf.EKF(x, q_true)
        
        # 5: Instantiate buffer file that data will be logged to. Populate first line of file with state vector initialization.
        colnames = ["t", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "q_scalar", "q_i", "q_j", "q_k"]
        self.data_logger = dl.DataLogger(x, q_true, colnames)
        
    def launch_detect_pyro_arm(self):
        """
        Launch Detect - Pyro Arm State
        - Waits for pyro arm to switch on, then begins Kalman Filtering
        - No data logging yet, wait until rocket is in the air
        """
        print("IN LD PA")
        
        # 1: Wait for pyro arm switch to be ON. Pyro arm can happen at any point before step 5. Step of 5 launch detect must be completed before LD-PA. There should be some boolean which stores pyro arm state. Pyro will be armed by a hardware switch.
        pyro_arm_on = True
        while pyro_arm_on is False:
            pyro_arm_on = check_pyro_arm()
            
        # 2: If step 1 is complete, read IMU and state vector continuously. If either the IMU detects an acceleration along Z axis of over (insert) for at least 3 samples or the velocity of the state vector is greater than (insert), Active Flight mode is triggered. (This condition needs to be tested and can be changed)
        # start kalman filtering here ?
        large_acceleration = False
        
        while large_acceleration == False:
            kalman_loop_no_logging(self.ekf, self.collector)
        
        
        
    def active_flight(self):
        """
        Active Flight State
        - Begin to log data, while still doing the Kalman Filtering
        - Stops once the acceleration stops
        """
        
        print("IN AF")
        
        # 1: Continuously log data to file buffer. Continuously monitor state vector Z velocity (ECEF) and altitude. If computer is in fixed altitude operator, deploy chute once that altitude is detected. If computer is in autodetect mode, fire chutes after .5 seconds of 3 state vector reading of Z velocity (ECEF) less than 0.5 m/s (test).
        self.data_logger.start_timer()
        
        # 2: If an acceleration of less than 0.5 m/s is detected for more that 5 seconds, end data logging, end program execution and save data file.
        large_acceleration = True
        while large_acceleration == True:
        
            kalman_loop_with_logging(self.ekf, self.collector, self.logger)
        
        self.collector.write_buffer_to_file()
  
# TODO: implement this once we have a pyro arm
def check_pyro_arm():
    return True     

def kalman_loop_no_logging(ekf, collector):
    """
    Run one instance of the Kalman loop. Does not write output data to a file

    Parameters
    ----------
    ekf : EKF class
        Implements an Extended Kalman Filter
    collector : SensorDataCollector class
        Reads data from sensors

    Returns
    -------
    None.

    """
    
    # Read IMU
    accel, gyro, dt = collector.get_next_imu_reading()
    z_imu = np.concatenate((accel, gyro))
    
    # Prediction
    ekf.predict(z_imu, dt)

    # Read GPS and barometer -- these return None if no new data
    baro = collector.get_next_barometer_reading()
    lla = collector.get_next_gps_reading()

    # Update
    baro = None
    ekf.update(lla, baro, sigma_gps=5, sigma_baro=10) # try variance = 10
    
def kalman_loop_with_logging(ekf, collector, logger):
    """
    Run one instance of the Kalman loop. Logs output data to a file

    Parameters
    ----------
    ekf : EKF class
        Implements an Extended Kalman Filter
    collector : SensorDataCollector class
        Reads data from sensors
    logger : DataLogger class
        Writes output data to a file

    Returns
    -------
    None.

    """
    
    # Read IMU
    accel, gyro, dt = collector.get_next_imu_reading()
    z_imu = np.concatenate((accel, gyro))
    
    # Prediction
    ekf.predict(z_imu, dt)

    # Read GPS and barometer -- these return None if no new data
    baro = collector.get_next_barometer_reading()
    lla = collector.get_next_gps_reading()

    # Update
    baro = None
    ekf.update(lla, baro, sigma_gps=5, sigma_baro=10) # try variance = 10
    
    # Log the data
    logger.save_state_to_buffer(ekf.x, ekf.q_e2b)

if __name__ == "__main__":
    
    fsm = StateMachine()
    
    # ------- state 1 ------- #
    fsm.launch_detect()
    
    # ------- state 2 ------- #
    fsm.launch_detect_pyro_arm()
    
    # ------- state 3 ------- #
    fsm.active_flight()
    