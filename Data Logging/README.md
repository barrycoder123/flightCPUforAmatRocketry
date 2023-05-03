README: Data Logging
Team Celestial Blue
Senior Design Project
Tufts University, 2023

The code in this directory implements data logging for our Flight Computer. This README explains the purpose of each file. See each file for more specifics. 

---------------------------------------------------------------------------------------------------

data_logger.py
- Implements a data logging class which writes output data to a file.

data_viewing.py
- Can be used to view the contents of a data file after navigation. The flight computer does not support plotting, so you'd want to copy the data file from the Flight Computer to your own PC, and then view it with this python script.

first_drive.csv and first_drive.png
- These are included as an example for you to run data_viewing.py. It will take the data from the .csv file and plot it over the map image in the .png file.

Test Results
- Folder which contains other CSV files and PNG files from our previous tests.
