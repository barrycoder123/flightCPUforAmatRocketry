import board
import adafruit_mpl3115a2

i2c = board.I2C()
baro = adafruit_mpl3115a2.MPL3115A2(i2c)

# Function getSLP gets the sealevel_pressure
def setSLP(newSLP):
    baro.sealevel_pressure = newSLP 

def getSLP():
    return baro.sealevel_pressure

def readALT():
    return baro.altitude
