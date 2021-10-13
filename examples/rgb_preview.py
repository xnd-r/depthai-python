#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.createColorCamera()
xoutRgb = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Linking
camRgb.preview.link(xoutRgb.input)

devConfig = dai.Device.Config()
if 1:
    devConfig.preboot.watchdogTimeoutMs = 0
    print("[CONFIG WARNING] Device watchdog disabled. A power-cycle is required"
          " before next run in case of unclean app exit")
if 1:
    devConfig.preboot.usb.maxSpeed = dai.UsbSpeed.HIGH
    print("[CONFIG WARNING] USB limited to high-speed (USB2)")

# Connect to the device
with dai.Device(devConfig) as device:
    # Print out available cameras
    print('Connected cameras: ', device.getConnectedCameras())
    # Print out usb speed
    print('Usb speed: ', device.getUsbSpeed().name)

    device.startPipeline(pipeline)

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inRgb = qRgb.get() # blocking call, will wait until a new data has arrived

        # Retrieve 'bgr' (opencv format) frame
        cv2.imshow("rgb", inRgb.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
