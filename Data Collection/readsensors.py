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

#setSLP(int(config.get('Barometer', 'sealevelpressure')))


last_val = 0xFFFF
# set up initial sensor values
last_accel = np.array([last_val,last_val,last_val])
last_gyro = np.array([last_val,last_val,last_val])
last_baro = last_val
last_time = time.perf_counter()

accel = last_accel
gyro = last_gyro
baro = last_baro

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

def collectdata(last_gyro, last_accel, last_baro, last_time, gyro, accel, baro):
     # global last_time
    
     last_gyro = read_gyro(last_gyro)
     last_accel = read_accel(last_accel)
     # last_baro = read_baro(last_baro)
     last_baro = 0
     time_step = time.perf_counter()-last_time
     last_time = last_time+time_step
    
     return last_gyro, last_accel, last_baro, time_step, last_time

# Code below is for debugging purposes:

if __name__ == '__main__':
    while True:

        last_gyro,last_accel,last_baro,time_step,last_time = collectdata(last_gyro, last_accel, last_baro, last_time, gyro, accel, baro)

        print("Accelerometer (m/s^2): {}".format(last_accel))
        print("Gyroscope (rad/sec): {}".format(last_gyro))
        print("Altitude (m): {}".format(last_baro))
        print("Time step (sec): {}".format(time_step))
        print()
    
        if (time_step < period):
            time.sleep(period-time_step)
