import json, nau7802py, pathlib, time

AVG_SIZE = 4
avgWeights = []

def recordSystemSettings(filename = 'systemSettings.json'):
    file = pathlib.Path(filename)
    pathlib.Path(file).mkdir(parents = True, exist_ok = True)
    
    # Get various values from the library and commit them to NVM
    settings = {'calibrationFactor': myScale.getCalibrationFactor(),
                'zeroOffset': myScale.getZeroOffset()}
                
    with open(file, 'w') as fh:
        fh.write(json.dumps(settings, indent = 4))

def calibrateScale():
    print('')
    print('')
    print('Scale calibration')
    
    _ = input('Setup scale with no weight on it. Press a key when ready.')

    myScale.calculateZeroOffset(64)    # Zero or Tare the scale. Average over 64 readings.
    print('New zero offset: ', end='')
    print(myScale.getZeroOffset())

    _ = input('Place known weight on scale. Press a key when weight is in place and stable.')

    weightOnScale = input("Please enter the weight, without units, currently sitting on the scale (for example '4.25'): ")
    print('')

    myScale.calculateCalibrationFactor(weightOnScale, 64)    # Tell the library how much weight is currently on it
    print('New cal factor: ', end='')
    print(round(myScale.getCalibrationFactor(), 2))

    print('New Scale Reading: ', end='')
    print(round(myScale.getWeight(), 2))

    recordSystemSettings()    # Commit these values to file

def readSystemSettings(filename = 'systemSettings.json'):
    file = pathlib.Path(filename)
    
    settingCalibrationFactor = 0
    settingZeroOffset = 1000
    
    if file.exists():
        with open(file, 'r') as fh:
            settings = json.load(fh)
            settingCalibrationFactor = settings['calibrationFactor']
            settingZeroOffset = settings['zeroOffset']

    # Pass these values to the library
    myScale.setCalibrationFactor(settingCalibrationFactor);
    myScale.setZeroOffset(settingZeroOffset);

    settingsDetected = True    # Assume for the moment that there are good cal values
    if settingCalibrationFactor < 0.1 or settingZeroOffset == 1000:
        settingsDetected = False    # Defaults detected. Prompt user to cal scale.
    return settingsDetected

myScale = nau7802py.NAU7802()

if myScale.begin():
    
    readSystemSettings()    # Load zeroOffset and calibrationFactor from file
    
    myScale.setSampleRate(nau7802py.NAU7802_SPS_Values['NAU7802_SPS_320'])    # Increase to max sample rate
    myScale.calibrateAFE()    # Re-cal analog front end when we change gain, sample rate, or channel
    
    print('Zero offset: ', end='')
    print(myScale.getZeroOffset())
    print('Calibration factor: ', end='')
    print(myScale.getCalibrationFactor())
    
    while True:
        currentReading = myScale.getReading()
        currentWeight = myScale.getWeight()
        
        print('Reading: ', end='')
        print(currentReading, end='')
        print('\tWeight: ', end='')
        print(round(currentWeight, 2))    # Print 2 decimal places
        
        avgWeights.append(currentWeight)
        if len(avgWeights) == AVG_SIZE:
            avgWeights.pop(0)
        
        avgWeight = 0
        for x in range(len(avgWeights)):
            avgWeight += avgWeights[x]
        avgWeight /= AVG_SIZE
        
        print('\tAvgWeight: ', end='')
        print(round(avgWeight, 2))    # Print 2 decimal places
        
        print('')
        time.sleep(2)
        
        #
        # Not yet implemented
        #
        # if settingsDetected == False:
            # print("\tScale not calibrated. Press 'c'.")
        
        # if (Serial.available())
        # {
            # byte incoming = Serial.read();

            # if (incoming == 't') //Tare the scale
                # myScale.calculateZeroOffset();
            # else if (incoming == 'c') //Calibrate
            # {
                # calibrateScale();
            # }
        # }