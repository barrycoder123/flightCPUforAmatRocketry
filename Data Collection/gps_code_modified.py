# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple GPS module demonstration.
# Will print NMEA sentences received from the GPS, great for testing connection
# Uses the GPS to send some commands, then reads directly from the GPS
import time
import board
import busio

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
import select
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


def ddm_to_dd(coord_str, cardinal_dir):
    """
    Convert a latitude or longitude reading from degrees and decimal minutes (DDM) to decimal degrees (DM)

    Parameters
    ----------
    - coord_str : DDM coordinate string of the form "dddmm.mmmmm..."
    - cardinal_dir : single capital char: "N", "S", "E", or "W"

    Returns
    -------
    - decimal_degrees : DD latitude or longitude in "+/- ddd.ddddd..."

    Notes
    -----
    - Returns None if bad input (such as empty string)

    """
    
    if (len(coord_str) == 0 or len(cardinal_dir) == 0):
        return None
    
    # split the coordinate string into degrees and minutes parts
    parts = coord_str.split(".")
    degrees = float(parts[0][:-2])
    minutes = float(parts[0][-2:] + '.' + parts[1][:4])
    
    # check if the coordinate is negative and adjust the sign accordingly
    if cardinal_dir in ['S', 'W']:
        sign = -1
    else:
        sign = 1
    
    # calculate the decimal degrees coordinate
    decimal_degrees = sign * (degrees + (minutes / 60.0))
    
    return decimal_degrees
   

def get_lla(data):
    """
    Extracts latitude, longitude, altitude, from data string

    Parameters
    ----------
    data_str : string
        data from gps.readline() that has been converted to string

    Returns
    -------
    lla : list
        - lat (float) latitude in decimal-degrees format
        - long (float) latitude in decimal-degrees format
        - alt (float) altitude in meters

    """
    lat = data[LATITUDE]
    long = data[LONGITUDE]
    alt = data[ALTITUDE]
    
    if '' in [lat, long, alt]:
        return None
    
    lat = ddm_to_dd(lat, data[NS])
    long = ddm_to_dd(long, data[EW])
    alt = float(alt)
       
    return [lat, long, alt]

last_time = time.monotonic()

def read_gps(num_desired_satellites=0, desired_update_time=GPS_UPDATE_TIME):
    """
    Reads the GPS
    
    Parameters
    ----------
    num_desired_satellites : int (optional)
        - the number of satellites that the GPS must lock on to
        - by default, this number is set to 0
    desired_update_time : float (optional)
        - the amount of time (seconds) that should elapse before the GPS provides a new reading
        - by default, this number is set to the GPS's inherent update time of 1 second. 
        You can increase it if you want less frequent updates, but if you decrease 
        it you will not get new data more frequently than one second

    Returns
    -------
    lla : list
        - lat (float) latitude in decimal-degrees format
        - long (float) latitude in decimal-degrees format
        - alt (float) altitude in meters

    Notes
    -----
    Returns None if:
        - the GPS cannot get a fix
        - the GPS gets a fix but no new data is available in the desired time frame
        - the GPS gets a fix but not enough satellites were used
        - the GPS data string is formatted poorly
    """
    global last_time

    gps.update() # update the GPS struct

    # sanity check: ensure GPS has a fix before moving on
    if not gps.has_fix:
        print("NO GPS FIX")
        return None

    # if not enough time has passed, you will not get new data
    current = time.monotonic()
    dt = current - last_time
    if dt < desired_update_time:
        return None # for when no data is available
    data = gps.readline()  # read dataline, times out after timeout defined in uart
    last_time = current

    # once we have data, format the data string
    data_str = "".join([chr(b) for b in data]) # convert to string from byteArray
    data_str = data_str.split(',') # separate into discrete elements using comma
    if len(data_str) < LONGITUDE:
        print("BAD DATA STRING")
        return None # for when the data string is too short
    
    # finally check if we've read enough satellites, for high accuracy
    if (gps.satellites is not None) and (int(gps.satellites) < num_desired_satellites):
        print("NOT ENOUGH SATELLITES")
        return None # for when we don't have enough satellites
    
    return get_lla(data_str)

def read_gps_new(num_desired_satellites=0, desired_update_time=GPS_UPDATE_TIME):
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
    
    return 0

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
        
        #lla = read_gps(num_desired_satellites=4, desired_update_time=1.0)
        lla = read_gps_new(num_desired_satellites=4, desired_update_time=1.0)

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
