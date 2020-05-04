import nau7802py, time

myScale = nau7802py.NAU7802()

if myScale.begin():
    while True:
        currentReading = myScale.getReading()
        print('Reading: ' + str(currentReading))
        time.sleep(2)