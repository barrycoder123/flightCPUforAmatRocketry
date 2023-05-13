README: Data Collection
Team Celestial Blue
Senior Design Project
Tufts University, 2023

The code in this directory implements data collection for our Flight Computer. This README explains the purpose of each file. See each file for more specifics. 


We have some higher-level classes which get data from one of two places: sensors or .csv files. We have a base class which exhibits polymorphism by calling one of two subclasses:

sensor_data_collector.py
- Implements a data collection subclass which reads from sensors in the "Sensor Code" folder.

file_data_collector.py
- Implements a data collection subclass which reads from .csv test files in the "Test Data" folder.

data_collection_wrapper.py
- Implements a data collection base class. The class can either reference file_data_collector.py if you want to read from test files, or sensor_data_collector.py if you want to read from your sensors. In the final product, we will omit file_data_collector.py, but it's nice to have for development.

---------------------------------------------------------------------------------------------------

Sensor Code Folder:

We have these files which deal with reading directly from the sensors:

read_gps.py
- Deals with reading directly from the GPS

read_imu.py
- Deals with reading directly from the IMU

read_baro.py:
- Deals with reading directly from the barometer (not implemented)

barometer.txt:
- A config file for calibrating the barometer

---------------------------------------------------------------------------------------------------

Test Data Folder:

This folder contains .csv files which store all of our test data.

---------------------------------------------------------------------------------------------------

From George:

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
