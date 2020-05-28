import nau7802py, time

myScale = nau7802py.NAU7802() # Create instance of the NAU7802 class

#
# Begin void setup() equivalent
#

print('Qwiic Scale Example')

if not myScale.begin():
    print('Scale not detected. Please check wiring. Freezing...')
    while True:
        pass
  
print('Scale detected!')

myScale.setGain(nau7802py.NAU7802_Gain_Values['NAU7802_GAIN_2']) # Gain can be set to 1, 2, 4, 8, 16, 32, 64, or 128.

myScale.setSampleRate(nau7802py.NAU7802_SPS_Values['NAU7802_SPS_40']) # Sample rate can be set to 10, 20, 40, 80, or 320Hz

myScale.calibrateAFE() # Does an internal calibration. Recommended after power up, gain changes, sample rate changes, or channel changes.

#
# Begin void loop() equivalent
#

while True:
    if myScale.available():
        currentReading = myScale.getReading();
        print('Reading: ', currentReading)
        time.sleep(0.1)
