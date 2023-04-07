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
def read_accel(last_accel):
    try:
        accel = np.array([float(item) for item in imu.acceleration])
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
    while True:

        # gather the data
        gyro, accel, baro, curr_time, dt = collectdata(gyro, accel, baro)
        
        # print the data
        print('=' * 40)
        print("Accelerometer (m/s^2): {}".format(accel))
        print("Gyroscope (rad/sec): {}".format(gyro))
        print("Altitude (m): {}".format(baro))
        print("Time: (sec): {}".format(curr_time))
        print("Time step (sec): {}".format(dt))
        print()

        # wait for 1 sec so we don't get flooded with data
        time.sleep(1)
