README: Testing/Validation
Team Celestial Blue
Senior Design Project
Tufts University, 2023

The code in this directory, "Testing/Validation," implements simulations and integration tests for our Flight Computer. 

The idea is that the simulations would be run on your PC for easy debugging/validation. Integration tests can be run directly on the Flight Computer, as opposed to your PC. This README explains the purpose of each file. See each file for more specifics. 

---------------------------------------------------------------------------------------------------

kalman_integration_test.py:
- Run this on the Flight Computer to test Kalman Filtering

strapdown_integration_test.py
- Run this on the Flight Computer to test the IMU Strapdown

kalman_simulation.py:
- Run this on your PC to verify/simulate the Kalman Filtering implementation

strapdown_simulation.py
- Run this on your PC to verify/simulate the IMU strapdown implementation

---------------------------------------------------------------------------------------------------

How to run:
- Open up your favorite Python IDE (we use Spyder) and run one of these scripts
- If using a text editor, you can run python3 [filename].py. no command line arguments