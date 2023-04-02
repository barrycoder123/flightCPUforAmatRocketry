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
uart = serial.Serial("/dev/ttyO2", baudrate=9600, timeout=10)

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
    - coord_str : coordinate string of the form "dddmm.mmmmm..."
    - cardinal_dir : single capital char: "N", "S", "E", or "W"

    Returns
    -------
    - decimal_degrees : latitude or longitude in "+/- ddd.ddddd..."

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
    

def get_llas(data_str):
    """
    Extracts latitude, longitude, altitude, and number of satellites from data string

    Parameters
    ----------
    data_str : string
        data from gps.readline() that has been converted to string

    Returns
    -------
    llas : list
        - lat (float) decimal degrees latitude
        - long (float) decimal degrees longitude
        - alt (float) altitude in meters
        - satellites (float) number of satellites

    """
    
    data = data_str.split(',')

    lat = ddm_to_dd(data[LATITUDE], data[NS])
    long = ddm_to_dd(data[LONGITUDE], data[EW])

    return [lat, long, gps.altitude_m, gps.satellites]


def read_gps():
    """
    Reads the GPS

    Returns
    -------
    llas : list
        - lat (float) decimal degrees latitude
        - long (float) decimal degrees longitude
        - alt (float) altitude in meters
        - satellites (float) number of satellites

    """
    
    # check if data is available from the serial port
    ready_to_read, _, _ = select.select([uart], [], [], 0)
    if ready_to_read:
        data = gps.readline()  # read dataline, times out after timeout defined in uart
    else:
        return None

    # convert bytearray to string
    data_str = "".join([chr(b) for b in data])
    return get_llas(data_str)

def read_gps_new():
    """
    potentially better function for reading GPS
    
    Returns
    -------
    llas : list
        - lat (float) decimal degrees latitude
        - long (float) decimal degrees longitude
        - alt (float) altitude in meters
        - satellites (float) number of satellites
        
    Notes
    -----
    Returns None if no new data is available
    """
    
    gps.update()
    
    # Return None if no available data
    if not gps.has_fix:
        return None
    
    # Extract relevant data otherwise
    lat = gps.latitude
    long = gps.longitude
    alt = gps.altitude_m
    satellites = gps.satellites
    
    return [lat, long, alt, satellites]
    

#get and ignore first output
gps.readline()

#default code provided by adafruit, which I wrapped in this if block
if __name__ == "__main__":
    
    # # Main loop runs forever printing data as it comes in
    last_print = time.monotonic()
    while True:
        
        # Make sure to call gps.update() every loop iteration and at least twice
        # as fast as data comes from the GPS unit (usually every second).
        # This returns a bool that's true if it parsed new data (you can ignore it
        # though if you don't care and instead look at the has_fix property).
        gps.update()
        # Every second print out current location details if there's a fix.
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print("Waiting for fix...")
            continue
        
        # We have a fix! (gps.has_fix is true)
        # Print out details about the fix like location, date, etc.
        print("=" * 40) # Print a separator line.
        print(
        "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
        gps.timestamp_utc.tm_mon, # Grab parts of the time from the
        gps.timestamp_utc.tm_mday, # struct_time object that holds
        gps.timestamp_utc.tm_year, # the fix time. Note you might
        gps.timestamp_utc.tm_hour, # not get all data like year, day,
        gps.timestamp_utc.tm_min, # month!
        gps.timestamp_utc.tm_sec,
        )
        )
        print("Latitude: {0:.6f} degrees".format(gps.latitude))
        print("Longitude: {0:.6f} degrees".format(gps.longitude))
        print("Fix quality: {}".format(gps.fix_quality))
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present. Check if they're None before trying to use!
        if gps.satellites is not None:
            print("# satellites: {}".format(gps.satellites))
        if gps.altitude_m is not None:
            print("Altitude: {} meters".format(gps.altitude_m))

        # MATT: uncomment this to run the code you have 
        # llas = read_gps()
        # if llas is not None:
        #     print(llas)