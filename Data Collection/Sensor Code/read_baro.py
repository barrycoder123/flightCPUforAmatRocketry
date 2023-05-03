import board
import pathlib
import configparser
import adafruit_mpl3115a2

i2c = board.I2C()
baro = adafruit_mpl3115a2.MPL3115A2(i2c)

path = pathlib.Path().resolve()
config = configparser.ConfigParser()
config.read(path / 'barometer.txt')

# setSLP(int(config.get('Barometer', 'sealevelpressure')))

# Function getSLP gets the sealevel_pressure
def setSLP(newSLP):
    baro.sealevel_pressure = newSLP 

def getSLP():
    return baro.sealevel_pressure

def readALT():
    return baro.altitude


def read_baro(last_baro):
    """
    Read the barometer
    
    Parameters:
        last_baro: (float) previous baro reading
        
    Returns:
        baro: (float) current baro reading
    """
    try:
        baro = float(readALT())
    except RuntimeError:
        print("error reading barometer, using previous value")
        baro = float(last_baro)
    return baro
