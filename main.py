import nau7802py, time

myScale = nau7802py.NAU7802()

if myScale.begin():
    while True:
        currentReading = myScale.getReading()
        print('Reading: ' + str(currentReading))
        time.sleep(2)
else:
    print("Something went wrong")

# Here's some blinkLED code if you need a Pymakr sanity check.
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