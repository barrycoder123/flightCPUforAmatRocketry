#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 18:08:19 2023

@author: zrummler
"""

import Adafruit_BBIO.GPIO as GPIO

BUTTON_PIN = "P9_23"

GPIO.setup(BUTTON_PIN, GPIO.IN)

def button_pressed():

    return GPIO.input(BUTTON_PIN) == GPIO.HIGH

if __name__ == "__main__":
    
    while True:
        if button_pressed():
            print("Button is on")
            #GPIO.output(LED_PIN, GPIO.LOW)
        else:
            print("Switch is off")
            #GPIO.output(LED_PIN, GPIO.HIGH)
