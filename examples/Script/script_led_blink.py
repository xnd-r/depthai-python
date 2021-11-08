#!/usr/bin/env python3

''' Blinks the device LED (GPIO 56), using GPIO control from `Script` node.
Note: on some devices, the LED is located on the SoM, and not externally visible
when inside the enclosure.
'''

import depthai as dai
import time

# Start defining a pipeline
pipeline = dai.Pipeline()

# Script node
script = pipeline.create(dai.node.Script)
script.setScript("""
    import GPIO
    import time

    ledGpio = 56
    ledOn = GPIO.HIGH
    GPIO.setup(ledGpio, GPIO.OUT)

    while True:
        GPIO.write(ledGpio, ledOn)
        time.sleep(0.5)
        GPIO.write(ledGpio, not ledOn)
        time.sleep(1.5)
""")

# Connect to device with pipeline
with dai.Device(pipeline) as device:
    # Nothing to do, just exit on Ctrl-C
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
