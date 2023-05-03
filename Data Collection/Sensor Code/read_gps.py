"""
Involves reading from the GPS

To use: 
    import read_gps
    
Functions:
    read_gps_lla()
"""

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple GPS module demonstration.
# Will print NMEA sentences received from the GPS, great for testing connection
# Uses the GPS to send some commands, then reads directly from the GPS
import time
import board
#import busio

import adafruit_gps

import Adafruit_BBIO.UART as UART

UART.setup("UART2")

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
# uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)

# for a computer, use the pyserial library for uart access
# use USBX for USB, e.g. USB0
# might need to change the timeout
# gps defaults to 9600 baud, don't change it
import serial
#import select
uart = serial.Serial("/dev/ttyO2", baudrate=9600, timeout=1)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart)  # Use UART/pyserial
# gps = adafruit_gps.GPS_GtopI2C(i2c)  # Use I2C interface

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA info (what you typically want)
gps.send_command(b"PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Turn on just minimum info (RMC only, location):
# gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
# gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on everything (not all of it is parsed!)
# gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

GPS_UPDATE_TIME = 1.0 # want the GPS to get new data every 1 second
# Set update rate to once a second (1hz) which is what you typically want.
# gps.send_command(b"PMTK220,1000")
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
# gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
# gps.send_command(b'PMTK220,500')

#these are the indices of each data point returned by the GPS
LATITUDE = 2
NS = 3
LONGITUDE = 4
EW = 5
SATELLITES = 7
ALTITUDE = 9
UNIT = 10


def read_gps_lla(num_desired_satellites=0, desired_update_time=GPS_UPDATE_TIME):
    """
    potentially better function for reading GPS
    potentially worse, so we'll have to test
    - as long as read_gps() is bug free I have no issue with it
    
    Parameters
    ----------
    num_desired_satellites : int
        - the number of satellites that the GPS must lock on to
        - by default, this number is set to 0
    desired_update_time : float 
        - the amount of time (seconds) that should elapse before the GPS provides a new reading
        - by default, this number is set to the GPS's inherent update time. 
        You can increase it if you want less frequent updates, but if you decrease 
        it you will not get new data each time
    
    Returns
    -------
    lla : list
        - lat (float) decimal degrees latitude
        - long (float) decimal degrees longitude
        - alt (float) altitude in meters
        
    Notes
    -----
    Returns None if:
        - the GPS cannot get a fix
        - the GPS gets a fix but no new data is available in the desired time frame
        - the GPS gets a fix but not enough satellites were used
    """
    global last_time#, gps

    # Update the GPS struct
    gps.update()

    # sanity check: ensure GPS has a fix before moving on
    #if not gps.has_fix:
    #    print("NO GPS FIX NEW")
    #    return None

    # if not enough time has passed, you will not get new data
    current = time.monotonic()
    dt = current - last_time
    if dt < desired_update_time:
        return None
    last_time = current

    # Return None if no available data
    if not gps.has_fix:
        print("NO FIX")
        return None
    
    if (gps.satellites is not None) and (int(gps.satellites) < num_desired_satellites):
        print("NOT ENOUGH SATELLITES")
        return None
    
    # Extract relevant data otherwise
    lat = gps.latitude
    long = gps.longitude
    alt = gps.altitude_m
    
    return [float(lat), float(long), float(alt)]
    

# default code provided by adafruit, which I wrapped in this if block
if __name__ == "__main__":
    
    # # Main loop runs forever printing data as it comes in
    while True:
        
        lla = read_gps_lla(num_desired_satellites=6, desired_update_time=1.0)

        # print data if we got some!
        if (lla is not None):
            
            # Print out details about the fix like location, date, etc.
            print("=" * 40) # Print a separator line.
            print(
            "Timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
            gps.timestamp_utc.tm_mon, # Grab parts of the time from the
            gps.timestamp_utc.tm_mday, # struct_time object that holds
            gps.timestamp_utc.tm_year, # the fix time. Note you might
            gps.timestamp_utc.tm_hour, # not get all data like year, day,
            gps.timestamp_utc.tm_min, # month!
            gps.timestamp_utc.tm_sec,
            )
            )
            print(f'lat: {lla[0]}, long: {lla[1]}, alt: {lla[2]}')
            #print(gps.latitude, gps.longitude, gps.altitude_m)
            #)print(f'casted lat2: {lla2[0]}, long: {lla2[1]}, alt: {lla2[2]}')
            print()
        #else:
            #print("Nothing")
