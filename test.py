import nau7802py, time

myScale = nau7802py.NAU7802()
# myScale.setGain(nau7802py.NAU7802_Gain_Values['NAU7802_GAIN_2'])

if myScale.begin():
    
    
    print('Zero Offset: ', myScale.getZeroOffset())
    myScale.calculateZeroOffset(100)
    print('Zero Offset set')
    print('Zero Offset: ', myScale.getZeroOffset())
    print('\n'*2)
    
    print('Calibration Factor: ', myScale.getCalibrationFactor())
    print('Place item on scale within 10 seconds')
    time.sleep(10)
    myScale.calculateCalibrationFactor(1000, 100)
    print('Calibration Factor: ', myScale.getCalibrationFactor())
    print('\n'*2)
    
    print('Testing getWeight')
    print('Place new item on scale within 10 seconds')
    time.sleep(10)
    print('Weight: ', myScale.getWeight())
    print('\n'*2)
#     
#     print('Testing in conversion ready pin polarity')
#     myScale.setIntPolarityLow()
#     print('Polarity low when ready')
#     c = 0
#     while c<10:
#         a = myScale.available()
#         print(a)
#         time.sleep(1)
#         c += 1
#     myScale.setIntPolarityHigh()
#     print('Polarity high when ready')
#     c = 0
#     while c<10:
#         a = myScale.available()
#         print(a)
#         time.sleep(1)
#         c += 1
#     print('\n'*2)
#     
#     print('Testing in power down')
#     print('Powered Down')
#     myScale.powerDown()
#     c = 0
#     while c<10:
#         a = myScale.available()
#         print(a)
#         time.sleep(1)
#         c += 1
#     print('Powered Up')
#     myScale.powerUp()
#     c = 0
#     while c<10:
#         a = myScale.available()
#         print(a)
#         time.sleep(1)
#         c += 1
#     print('\n'*2)
#     
#     print('Testing in channel selection')
#     print('Channel 2')
#     myScale.setChannel(2)
#     c = 0
#     while c<10:
#         a = myScale.getReading()
#         print(a)
#         time.sleep(1)
#         c += 1
#     print('Channel 1')
#     myScale.setChannel(1)
#     c = 0
#     while c<10:
#         a = myScale.getReading()
#         print(a)
#         time.sleep(1)
#         c += 1
#     print('\n'*3)
#     
#     while True:
#         currentReading = myScale.getReading()
#         print('Reading: ' + str(currentReading))
#         time.sleep(2)