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

# Setup initial sensor values
last_val = 0xFFFF
accel = np.array([last_val,last_val,last_val])
gyro = np.array([last_val,last_val,last_val])
baro = last_val

# Setup sample rate
sample_rate_Hz = 50
period = 1/sample_rate_Hz

# Setup calibration offsets
accel_offsets = np.zeros(3)
gyro_offsets = np.zeros(3)

# Function Definitions

def calibrate_imu():
    """
    Calibrates the IMU after powerup
    
    Effects:
        Sets accel_offsets and gyro_offsets global variables
    """
    global accel_offsets, gyro_offsets
    
    # Only needs to be calibrated once upon powerup
    if imu.calibrated:
        return
    
    # Collect calibration data for each position
    print('Calibrating the IMU sensor...')
    while True:
        sys, gyro, accel, mag = imu.calibration_status
        if gyro == 3 and accel == 3:
            break
        elif gyro == 3:
            print("Gyroscope calibrated!")
            print("Move the sensor in different positions to calibrate accelerometer...")
        elif accel == 3:
            print("Accelerometer calibrated!")
            print("Keep the sensor steady to calibrate gyroscope...")
        else:
            print("Neither accel/gyro have been calibrated...")
        
        time.sleep(0.5)
            
    print('IMU has been calibrated!')
    accel_offsets = [float(data) for data in imu.offsets_accelerometer]
    gyro_offsets = [float(data) for data in imu.offsets_accelerometer]
    
    print("Acceleration offsets:",accel_offsets)
    print("Gyroscope offsets:",gyro_offsets)
            
            
def read_accel(last_accel):
    """
    Read and return the acceleration vector from the IMU
    
    Parameters:
        last_accel: (3,) previous accel reading, assume it has been calibrated
        
    Returns:
        calibrated_accel: (3,) current calibrated accel reading
    """
    try:
        calibrated_accel = [float(data) for data in imu.acceleration]
        #calibrated_accel = accel_reading - accel_offsets
        calibrated_accel = np.array(calibrated_accel)
        
    except RuntimeError:
        print("error reading acceleration, using previous value")
        calibrated_accel = last_accel
        
    return calibrated_accel


def read_gyro(last_gyro):
    """
    Read and return the gyro vector from the IMU
    
    Parameters:
        last_gyro: (3,) previous gyro reading, assume it has been calibrated
        
    Returns:
        calibrated_gyro: (3,) current calibrated gyro reading
    """
    try:
        calibrated_gyro = [float(data) for data in imu.gyro]
        #calibrated_gyro = gyro_reading - gyro_offsets
        calibrated_gyro = np.array(calibrated_gyro)
        
    except RuntimeError:
        print("error reading gyro, using previous value")
        calibrated_gyro = last_gyro

    return calibrated_gyro


def read_baro(last_baro):
    """
    Read the barometer
    
    Parameters:
        last_baro: (float) previous baro reading
        
    Returns:
        baro: (float) current baro reading
    """
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
