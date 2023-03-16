IMU and Barometer Code for Team Celestial Blue Senior Design Project
George Eng

To use, include "from readsensors import *" at the top of the file.

make sure to install the following libraries via "pip install []":
pyserial
adafruit_bno055
board
adafruit_circuitpython_mpl3115a2
configparser

The IMU and Barometer data updates as fast as possible and gives the time step between each measurement.

The daily sea-level air pressure must be specified in pascals (Pa) in the config file included in the directory.
Line 13 of readsensors.py must contain the full path of the config file 'barometer.txt'

Due to issues with the raspberry pi not working (as of 3.13.2023), I have not been able to test the code works after adding the config file reading.
