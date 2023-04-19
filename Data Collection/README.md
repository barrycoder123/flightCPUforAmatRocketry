README: Data Collection
Team Celestial Blue
Senior Design Project
Tufts University, 2023

The code in this directory implements data collection for our Flight Computer. This README explains the purpose of each file. See each file for more specifics. 

---------------------------------------------------------------------------------------------------

We have three files which deal with reading directly from the sensors:

gps_code_modified:
- Deals with reading directly from the GPS

readsensors.py
- Deals with reading directly from the barometer and IMU

barometer.py:
- Helper file for readsensors.py

barometer.txt:
- A config file for calibrating the barometer

---------------------------------------------------------------------------------------------------

Additionally, we have some higher-level classes which act as wrappers for the functions listed above. We have a base class which exhibits polymorphism by calling one of two subclasses:

sensor_data_collector.py
- Implements a data collection subclass which reads from sensors. 

file_data_collector.py
- Implements a data collection subclass which reads from test files.

data_collection_wrapper.py
- Implements a data collection base class. The class can either reference file_data_collector.py if you want to read from test files, or sensor_data_collector.py if you want to read from your sensors. In the final product, we will omit file_data_collector.py, but it's nice to have for development.

I am thinking about combining the three of these into one file for simplicity.

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
