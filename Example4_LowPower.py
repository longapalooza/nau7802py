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

#
# Begin void loop() equivalent
#

while True:
    myScale.powerDown() # Power down to ~200nA
    time.sleep(1)
    myScale.powerUp() # Power up scale. This scale takes ~600ms to boot and take reading.

    # Time how long it takes for scale to take a reading
    startTime = time.time()
    while not myScale.available():
        time.sleep(0.001)

    currentReading = myScale.getReading();
    print('Startup time: ', time.time() - startTime, end = '')
    print(', ', currentReading)
    time.sleep(0.1)
