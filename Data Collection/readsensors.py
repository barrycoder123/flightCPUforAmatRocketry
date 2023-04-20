"""
For calibrating the IMU:

Gyroscope: The device must be standing still in any position

Accelerometer: The BNO055 must be placed in 6 standing positions for +X, -X, +Y, 
-Y, +Z and -Z. This is the most onerous sensor to calibrate, but the best solution 
to generate the calibration data is to find a block of wood or similar object, 
and place the sensor on each of the 6 'faces' of the block, which will help to 
maintain sensor alignment during the calibration process. You should still be 
able to get reasonable quality data from the BNO055, however, even if the 
accelerometer isn't entirely or perfectly calibrated.
"""


import serial
import adafruit_bno055
import time
#from barometer import *
import configparser
import Adafruit_BBIO.UART as UART
import pathlib
import numpy as np

UART.setup("UART1")

# Begin Setup
uart = serial.Serial("/dev/ttyO1", 115200)
imu = adafruit_bno055.BNO055_UART(uart)

path = pathlib.Path().resolve()
config = configparser.ConfigParser()
config.read(path / 'barometer.txt')

# setSLP(int(config.get('Barometer', 'sealevelpressure')))

last_val = 0xFFFF
# set up initial sensor values
accel = np.array([last_val,last_val,last_val])
gyro = np.array([last_val,last_val,last_val])
baro = last_val

sample_rate_Hz = 50
period = 1/sample_rate_Hz



# Function Definitions

def calibrate_imu():
    print('Calibrating the BNO055 sensor...')
    while True:
        sys, gyro, accel, mag = imu.get_calibration_status()
        if gyro == 3 and accel == 3:
            break
        else:
            print('Move the sensor in different positions...')
            time.sleep(1)
            
            
def read_accel(last_accel):
    try:
        #accel = np.array([0, 0, 9.81])
        accel = np.array([float(item) for item in imu.gravity])
    except RuntimeError:
        print("error reading acceleration, using previous value")
        accel = last_accel
    return accel
def read_gyro(last_gyro):
    try:
        gyro = np.array([float(item) for item in imu.gyro])
    except RuntimeError:
        print("error reading gyroscope, using previous value")
        gyro = last_gyro
    return gyro
def read_baro(last_baro):
    try:
        baro = float(readALT())
    except RuntimeError:
        print("error reading barometer, using previous value")
        baro = float(last_baro)
    return baro

def read_quat():
    try: 
        quat = np.array(imu.quaternion)
    except RuntimeError:
        print("error reading quaternion")
        quat = None

    return quat

start_time = time.perf_counter()
last_time = start_time
def collectdata(gyro, accel, baro):
     global last_time
    
     gyro = read_gyro(gyro)
     accel = read_accel(accel)
     # last_baro = read_baro(last_baro)
     baro = 0
     current_time = time.perf_counter() - start_time
     dt = current_time - last_time
     last_time = current_time
     
     return gyro, accel, baro, current_time, dt

# Code below is for debugging purposes:

if __name__ == '__main__':
    
    calibrate_imu()
    
    while True:

        # gather the data
        gyro, accel, baro, curr_time, dt = collectdata(gyro, accel, baro)
        
        # print the data
        print('=' * 40)
        print("Accelerometer (m/s^2): {}".format(accel))
        print("Gyroscope (rad/sec): {}".format(gyro))
        print("Altitude (m): {}".format(baro))
        print("Quaternion:", read_quat())
        print("Time: (sec): {}".format(curr_time))
        print("Time step (sec): {}".format(dt))
        print()

        # wait for 1 sec so we don't get flooded with data
        time.sleep(1)
