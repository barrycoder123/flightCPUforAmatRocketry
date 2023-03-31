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
gps.send_command(b"PMTK220,1000")
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
# gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
# gps.send_command(b'PMTK220,500')

#these are the indices of each data point returned by the GPS
LATITUDE = 2
LONGITUDE = 4
SATELLITES = 7
ALTITUDE = 9
UNIT = 10

#extracts latitude, longitude, altitude, and #satellites
def get_llas(data_str):
    data = data_str.split(',')
    return [data[i] for i in [2, 4, 9, 7]]

#just returns altitude
def get_altitude(data_str):
    data = data_str.spit(',')
    return data[ALTITUDE]

#returns [lat, long, alt, #satellites]
def read_gps():
    data = gps.readline()  # read dataline, times out after timeout defined in uart
    # print(data)  # this is a bytearray type

    if data is not None:
        # convert bytearray to string
        data_str = "".join([chr(b) for b in data])
        # print(data_str, end="")
        return get_llas(data_str)
    return None
    
#default code provided by adafruit, which I wrapped in this if block

if __name__ == "__main__":
    # Main loop runs forever printing data as it comes in
    timestamp = time.monotonic()
    while True:
        data = gps.read(32)  # read up to 32 bytes
        # print(data)  # this is a bytearray type

        if data is not None:
            # convert bytearray to string
            data_string = "".join([chr(b) for b in data])
            print(data_string, end="")

        # if time.monotonic() - timestamp > 5:
        #     # every 5 seconds...
        #     gps.send_command(b"PMTK605")  # request firmware version
        #     timestamp = time.monotonic()
