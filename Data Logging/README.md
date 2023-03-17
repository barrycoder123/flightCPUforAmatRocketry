README
Team Celestial Blue
Senior Design Project
Tufts University, 2023

The code in this repository implements data logging for our Flight Computer. This README explains the purpose of each file. See each file for more specifics. 

---------------------------------------------------------------------------------------------------

We have a base class which exhibits polymorphism by calling one of two subclasses:

flash_writer.py
- Implements a data logging subclass which writes output data to a file.

data_plotter.py
- Implements a data logging subclass which displays output data via plotting.

data_collection_wrapper.py
- Implements a data logging base class. The class can either reference flash_writer.py if you want to write your data to a file, or data_logger.py if you want to visualize your data. In the final product, we will omit plotting, but it's nice to have in the development stage.

I am thinking about combining the three of these into one file for simplicity.

---------------------------------------------------------------------------------------------------
