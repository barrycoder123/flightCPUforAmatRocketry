#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 11:36:35 2023

@author: zrummler

From the data sheet:

Gyroscope: The device must be standing still in any position

Accelerometer: The BNO055 must be placed in 6 standing positions for +X, -X, +Y, 
-Y, +Z and -Z. This is the most onerous sensor to calibrate, but the best solution 
to generate the calibration data is to find a block of wood or similar object, 
and place the sensor on each of the 6 'faces' of the block, which will help to 
maintain sensor alignment during the calibration process. You should still be 
able to get reasonable quality data from the BNO055, however, even if the 
accelerometer isn't entirely or perfectly calibrated.
"""

from readsensors import *


def calibrate_imu():
    print('Calibrating the BNO055 sensor...')
    while True:
        sys, gyro, accel, mag = imu.get_calibration_status()
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            break
        else:
            print('Move the sensor in different positions...')
            time.sleep(1)