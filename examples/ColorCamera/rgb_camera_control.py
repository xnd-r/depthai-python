#!/usr/bin/env python3

"""
This example shows usage of Camera Control message as well as ColorCamera configInput to change crop x and y
Uses 'WASD' controls to move the crop window, 'C' to capture a still image, 'T' to trigger autofocus, 'IOKL,.'
for manual exposure/focus:
  Control:      key[dec/inc]  min..max
  exposure time:     I   O      1..33000 [us]
  sensitivity iso:   K   L    100..1600
  focus:             ,   .      0..255 [far..near]
To go back to auto controls:
  'E' - autoexposure
  'F' - autofocus (continuous)
"""

import cv2
import datetime
import os

import depthai as dai

from os.path import join


# Step size ('W','A','S','D' controls)
STEP_SIZE = 8
# Manual exposure/focus set step
EXP_STEP = 100  # us
ISO_STEP = 10
LENS_STEP = 3

def clamp(num, v0, v1):
    return max(v0, min(num, v1))

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
videoEncoder = pipeline.create(dai.node.VideoEncoder)
stillEncoder = pipeline.create(dai.node.VideoEncoder)

controlIn = pipeline.create(dai.node.XLinkIn)
configIn = pipeline.create(dai.node.XLinkIn)
videoMjpegOut = pipeline.create(dai.node.XLinkOut)
stillMjpegOut = pipeline.create(dai.node.XLinkOut)
previewOut = pipeline.create(dai.node.XLinkOut)

controlIn.setStreamName('control')
configIn.setStreamName('config')
videoMjpegOut.setStreamName('video')
stillMjpegOut.setStreamName('still')
previewOut.setStreamName('preview')

# Properties
camRgb.setVideoSize(1920, 1080)
camRgb.setPreviewSize(640, 640)
videoEncoder.setDefaultProfilePreset(camRgb.getVideoSize(), camRgb.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)
stillEncoder.setDefaultProfilePreset(camRgb.getStillSize(), 1, dai.VideoEncoderProperties.Profile.MJPEG)

# Linking
camRgb.video.link(videoEncoder.input)
camRgb.still.link(stillEncoder.input)
camRgb.preview.link(previewOut.input)
controlIn.out.link(camRgb.inputControl)
configIn.out.link(camRgb.inputConfig)
videoEncoder.bitstream.link(videoMjpegOut.input)
stillEncoder.bitstream.link(stillMjpegOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    save_folder = "."
    os.makedirs(join(save_folder, 'records'), exist_ok=True)
    folder_name = join(save_folder, 'records', datetime.datetime.now().strftime('%m_%d_%H_%M_%S'))
    os.makedirs(folder_name, exist_ok=True)

    # Get data queues
    controlQueue = device.getInputQueue('control')
    configQueue = device.getInputQueue('config')
    previewQueue = device.getOutputQueue('preview')
    videoQueue = device.getOutputQueue('video')
    stillQueue = device.getOutputQueue('still')

    # Max cropX & cropY
    maxCropX = (camRgb.getResolutionWidth() - camRgb.getVideoWidth()) / camRgb.getResolutionWidth()
    maxCropY = (camRgb.getResolutionHeight() - camRgb.getVideoHeight()) / camRgb.getResolutionHeight()

    # Default crop
    cropX = 0
    cropY = 0
    sendCamConfig = True

    # Defaults and limits for manual focus/exposure controls
    lensPos = 150
    lensMin = 0
    lensMax = 255

    expTime = 20000
    expMin = 1
    expMax = 33000

    sensIso = 800
    sensMin = 100
    sensMax = 1600

    # Constants for preview overlay
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (0, 0, 255)
    thickness = 2
    line = cv2.LINE_AA

    # Frame frops are possible
    with open(os.path.join(folder_name, 'color.h265'), 'wb') as fileColorH265:
        while True:
            previewFrames = previewQueue.tryGetAll()
            for previewFrame in previewFrames:
                # Get exposure&iso
                exposure_time = previewFrame.getExposureTime()
                sensitivity = previewFrame.getSensitivity()

                frame = previewFrame.getData().reshape(previewFrame.getWidth(), previewFrame.getHeight(), 3)
                cv2.putText(frame, "Exposure: {}".format(exposure_time),
                            (50, 50), font, fontScale, color, thickness, line)
                cv2.putText(frame, "Sensitivity: {}".format(sensitivity),
                            (50, 100), font, fontScale, color, thickness, line)

                cv2.imshow('preview', frame)

            videoFrames = videoQueue.tryGetAll()
            for videoFrame in videoFrames:
                # Decode JPEG
                frame_data = videoFrame.getData()
                frame = cv2.imdecode(frame_data, cv2.IMREAD_UNCHANGED)
                frame_data.tofile(fileColorH265)

                # Display
                cv2.imshow('video', frame)

                # Send new cfg to camera
                if sendCamConfig:
                    cfg = dai.ImageManipConfig()
                    cfg.setCropRect(cropX, cropY, 0, 0)
                    configQueue.send(cfg)
                    print('Sending new crop - x: ', cropX, ' y: ', cropY)
                    sendCamConfig = False

            stillFrames = stillQueue.tryGetAll()
            for stillFrame in stillFrames:
                # Decode JPEG
                frame = cv2.imdecode(stillFrame.getData(), cv2.IMREAD_UNCHANGED)
                # Display
                cv2.imshow('still', frame)

            # Update screen (1ms pooling rate)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('c'):
                ctrl = dai.CameraControl()
                ctrl.setCaptureStill(True)
                controlQueue.send(ctrl)
            elif key == ord('t'):
                print("Autofocus trigger (and disable continuous)")
                ctrl = dai.CameraControl()
                ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
                ctrl.setAutoFocusTrigger()
                controlQueue.send(ctrl)
            elif key == ord('f'):
                print("Autofocus enable, continuous")
                ctrl = dai.CameraControl()
                ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
                controlQueue.send(ctrl)
            elif key == ord('e'):
                ctrl = dai.CameraControl()
                ctrl.setAutoExposureEnable()
                print("Autoexposure enable. Exposure: {}, Sensitivity: {}".format(
                    ctrl.getExposureTime(), ctrl.getSensitivity()))
                controlQueue.send(ctrl)
            elif key in [ord(','), ord('.')]:
                if key == ord(','): lensPos -= LENS_STEP
                if key == ord('.'): lensPos += LENS_STEP
                lensPos = clamp(lensPos, lensMin, lensMax)
                print("Setting manual focus, lens position: ", lensPos)
                ctrl = dai.CameraControl()
                ctrl.setManualFocus(lensPos)
                controlQueue.send(ctrl)
            elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
                if key == ord('i'): expTime -= EXP_STEP
                if key == ord('o'): expTime += EXP_STEP
                if key == ord('k'): sensIso -= ISO_STEP
                if key == ord('l'): sensIso += ISO_STEP
                expTime = clamp(expTime, expMin, expMax)
                sensIso = clamp(sensIso, sensMin, sensMax)
                print("Setting manual exposure, time: ", expTime, "iso: ", sensIso)
                ctrl = dai.CameraControl()
                ctrl.setManualExposure(expTime, sensIso)
                controlQueue.send(ctrl)
            elif key in [ord('w'), ord('a'), ord('s'), ord('d')]:
                if key == ord('a'):
                    cropX = cropX - (maxCropX / camRgb.getResolutionWidth()) * STEP_SIZE
                    if cropX < 0: cropX = maxCropX
                elif key == ord('d'):
                    cropX = cropX + (maxCropX / camRgb.getResolutionWidth()) * STEP_SIZE
                    if cropX > maxCropX: cropX = 0
                elif key == ord('w'):
                    cropY = cropY - (maxCropY / camRgb.getResolutionHeight()) * STEP_SIZE
                    if cropY < 0: cropY = maxCropY
                elif key == ord('s'):
                    cropY = cropY + (maxCropY / camRgb.getResolutionHeight()) * STEP_SIZE
                    if cropY > maxCropY: cropY = 0
                sendCamConfig = True
cmd = "ffmpeg -i {} {}"
os.system(cmd.format(join(folder_name, "color.h265"), join(folder_name, "color.mp4")))
