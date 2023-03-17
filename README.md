README
Team Celestial Blue
Senior Design Project
Tufts University, 2023

The code in this repository implements the software for our Flight Computer. This README explains the purpose of each folder. See each folder for more specific info.

Integration:
- Contains code that runs on the actual flight computer, as opposed to a PC

Simulations:
- Contains code that can only run on a PC, good for quick debugging/validation

Flight Algorithms:
- Contains code for Kalman filtering, the IMU strapdown, a quaternion library, and Earth model (reference frames and conversions)

Data Collection:
- Contains code which deals with getting input data from sensors (and data files)

Data Logging:
- Contains code which deals with saving output data to CSV files

Test Data:
- Contains files which store sample test data (from Tyler and Ian)

Unit Tests:
- Contains a few unit tests