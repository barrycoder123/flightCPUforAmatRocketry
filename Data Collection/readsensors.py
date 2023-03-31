import serial
import adafruit_bno055
import time
from barometer import *
import configparser

# Begin Setup
uart = serial.Serial("/dev/serial0")
imu = adafruit_bno055.BNO055_UART(uart)

config = configparser.ConfigParser()
config.read(r'C:\Users\geng\Downloads\rocketproject\barometer.txt')

setSLP(int(config.get('Barometer', 'sealevelpressure')))


last_val = 0xFFFF
# set up initial sensor values
last_accel = last_val
last_gyro = last_val
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
        accel = imu.acceleration
    except RuntimeError:
        print("error reading acceleration, using previous value")
        accel = last_accel
    return accel
def read_gyro(last_gyro):
    try:
        gyro = imu.gyro
    except RuntimeError:
        print("error reading gyroscope, using previous value")
        gyro = last_gyro
    return gyro
def read_baro(last_baro):
    try:
        baro = readALT()
    except RuntimeError:
        print("error reading barometer, using previous value")
        baro = last_baro
    return baro

# def collectdata(last_gyro, last_accel, last_baro, last_time, gyro, accel, baro):
#     global last_time
    
#     last_gyro = read_gyro(last_gyro)
#     last_accel = read_accel(last_accel)
#     last_baro = read_baro(last_baro)
#     time_step = time.perf_counter()-last_time
#     last_time = last_time+time_step
    
#     return last_gyro, last_accel, last_baro, time_step, last_time

# Code below is for debugging purposes:

#while True:
#
#    last_gyro,last_accel,last_baro,time_step,last_time = collectdata(last_gyro, last_accel, last_baro, last_time, gyro, accel, baro)
#
#    print("Accelerometer (m/s^2): {}".format(last_accel))
#    print("Gyroscope (rad/sec): {}".format(last_gyro))
#    print("Altitude (m): {}".format(last_baro))
#    print("Time step (sec): {}".format(time_step))
#    print()
#    
#    if (time_step < period):
#        time.sleep(period-time_step)