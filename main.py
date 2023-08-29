import nau7802py, time

# print('Got to line 3')

myScale = nau7802py.NAU7802()

# print('Got to line 7')

if myScale.begin(): #ok so the code fails on this
    while True:
        currentReading = myScale.getReading()
        print('Reading: ' + str(currentReading))
        time.sleep(2)

else:
    print('myScale.begin() failed')
# import json, nau7802py, pathlib, time

# myScale = nau7802py.NAU7802() # Create instance of the NAU7802 class

# settingsDetected = False # Used to prompt user to calibrate their scale

# # Create an array to take average of weights. This helps smooth out jitter.
# AVG_SIZE = 20
# avgWeights = []
# avgWeightSpot = 0

# # Gives user the ability to set a known weight on the scale and calculate a calibration factor
# def calibrateScale():
#     print('')
#     print('')
#     print('Scale calibration')
    
#     _ = input('Setup scale with no weight on it. Press a key when ready.')

#     myScale.calculateZeroOffset(64)    # Zero or Tare the scale. Average over 64 readings.
#     print('New zero offset: ', myScale.getZeroOffset())

#     _ = input('Place known weight on scale. Press a key when weight is in place and stable.')

#     weightOnScale = input("Please enter the weight, without units, currently sitting on the scale (for example '4.25'): ")
#     print('')

#     myScale.calculateCalibrationFactor(float(weightOnScale), 64)    # Tell the library how much weight is currently on it
#     print('New cal factor: ', round(myScale.getCalibrationFactor(), 2))

#     print('New Scale Reading: ', round(myScale.getWeight(), 2))

#     recordSystemSettings()    # Commit these values to file

# # Record the current system settings to NVM
# def recordSystemSettings(filename = 'systemSettings.json'):
#     file = pathlib.Path(filename)
#     pathlib.Path(file).touch()
    
#     # Get various values from the library and commit them to NVM
#     settings = {'calibrationFactor': myScale.getCalibrationFactor(),
#                 'zeroOffset': myScale.getZeroOffset()}
                
#     with open(file, 'w') as fh:
#         fh.write(json.dumps(settings, indent = 4))
    
#     global settingsDetected
#     settingsDetected = True

# # Reads the current system settings from NVM
# # If anything looks weird, reset setting to default value
# def readSystemSettings(filename = 'systemSettings.json'):
#     file = pathlib.Path(filename)
    
#     settingCalibrationFactor = 0
#     settingZeroOffset = 1000
    
#     if file.exists():
#         with open(file, 'r') as fh:
#             settings = json.load(fh)
#             settingCalibrationFactor = settings['calibrationFactor']
#             settingZeroOffset = settings['zeroOffset']

#     # Pass these values to the library
#     myScale.setCalibrationFactor(settingCalibrationFactor);
#     myScale.setZeroOffset(settingZeroOffset);
    
#     global settingsDetected
#     settingsDetected = True    # Assume for the moment that there are good cal values
#     if settingCalibrationFactor < 0.1 or settingZeroOffset == 1000:
#         settingsDetected = False    # Defaults detected. Prompt user to cal scale.
#     return settingsDetected

# #
# # Begin void setup() equivalent
# #

# print('Qwiic Scale Example')

# if not myScale.begin():
#     print('Scale not detected. Please check wiring. Freezing...')
#     while True:
#         pass

# print('Scale detected!')

# readSystemSettings()    # Load zeroOffset and calibrationFactor from file

# myScale.setSampleRate(nau7802py.NAU7802_SPS_Values['NAU7802_SPS_320'])    # Increase to max sample rate
# myScale.calibrateAFE()    # Re-cal analog front end when we change gain, sample rate, or channel

# print('Zero offset: ', myScale.getZeroOffset())
# print('Calibration factor: ', myScale.getCalibrationFactor())

# #
# # Begin void loop() equivalent
# #

# while True:
    
#     if myScale.available():
#         currentReading = myScale.getReading()
#         currentWeight = myScale.getWeight()
        
#         print('Reading: ', currentReading, end = '')
#         print('\tWeight: ', round(currentWeight, 2), end = '')    # Print 2 decimal places
        
#         avgWeights.append(currentWeight)
#         if len(avgWeights) == AVG_SIZE:
#             avgWeights.pop(0)
        
#         avgWeight = 0
#         for x in range(len(avgWeights)):
#             avgWeight += avgWeights[x]
#         avgWeight /= AVG_SIZE
        
#         print('\tAvgWeight: ', round(avgWeight, 2))    # Print 2 decimal places
        
#         if settingsDetected == False:
#             r = input("\tScale not calibrated. Press 'c'. ")
#             if r == 'c':
#                 calibrateScale()
        
#         else:
#             r = input('Calibrate (c), Tare (t), or Read (r) ')
#             if r == 'c':
#                 calibrateScale() # Calibrate
#             elif r == 't':
#                 myScale.calculateZeroOffset() # Tare the scale
        
#         print('')
#         time.sleep(2)



# import machine #you can ignore the yellow underline here
# import time

# def toggle_led(t):
#     led_pin.value(not led_pin.value())

# led_pin = machine.Pin(2, machine.Pin.OUT)
# led_timer = machine.Timer(1)
# led_timer.init(mode=machine.Timer.PERIODIC, period=1000, callback=toggle_led)


# while True:
#     led_pin.value(1)
#     print("ON...")
#     time.sleep(1)
#     led_pin.value(0)
#     print("OFF...")
#     time.sleep(1)