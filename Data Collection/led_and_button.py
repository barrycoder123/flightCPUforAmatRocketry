#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 18:08:19 2023

@author: zrummler
"""

import Adafruit_BBIO.GPIO as GPIO

BUTTON_PIN = "PIN_NAME"
LED_PIN = "P8_10"

GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN)


if __name__ == "__main__":
    
    
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            print("Button is off")
            GPIO.output(LED_PIN, GPIO.LOW)
        else:
            print("Switch is on")
            GPIO.output(LED_PIN, GPIO.HIGH)