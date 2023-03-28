#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 15:18:26 2023

@author: zrummler
"""

import platform

class DataLogger:
    
    def __init__(self, num_points):
        self.num_points = num_points
    
    def create(self):
        
        # For Zack on a Mac
        if platform.processor() == 'i386':
            return self.__data_plotter(self.num_points)
        
        # Print welcome messages and instructions
        print("Welcome to the Data Logging module!") 
        print("You are running this code on an unrecognized computer. It's likely that you're running on the BeagleBoard for the first time, or you're on a new PC.\n")
        print("\tSelect (1) if you want your results to be plotted in your window\n")
        print("\tSelect (2) if you want your data to be written to flash memory\n")
        
        # Take input until you get 1 or 2
        val = 0
        while (val != 1) and (val != 2):
            val = int(input(">> "))
            
        # Choose subclass based on selection
        if val == 1:
            return self.__data_plotter(self.num_points)
        if val == 2:
            return self.__flash_writer()
            
    def save(self):
        raise NotImplementedError("Did you forget to call DataLogger().create() ?")
    
    def show(self):
        raise NotImplementedError("Did you forget to call DataLogger().create() ?")
           
    @classmethod
    def __data_plotter(self, num_points):
        from data_plotter import DataPlotter
        return DataPlotter(num_points)
    
    @classmethod
    def __flash_writer(self, num_points):
        from flash_writer import FlashWriter
        return FlashWriter(num_points)