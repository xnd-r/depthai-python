#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time

'''
Spatial detection network demo.
    Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
'''
class StereoConfigHandler:

    class Trackbar:
        def __init__(self, trackbarName, windowName, minValue, maxValue, defaultValue, handler):
            self.min = minValue
            self.max = maxValue
            self.windowName = windowName
            self.trackbarName = trackbarName
            cv2.createTrackbar(trackbarName, windowName, minValue, maxValue, handler)
            cv2.setTrackbarPos(trackbarName, windowName, defaultValue)

        def set(self, value):
            if value < self.min:
                value = self.min
                print(f'{self.trackbarName} min value is {self.min}')
            if value > self.max:
                value = self.max
                print(f'{self.trackbarName} max value is {self.max}')
            cv2.setTrackbarPos(self.trackbarName, self.windowName, value)

    newConfig = False
    config = None
    trSigma = list()
    trConfidence = list()
    trLrCheck = list()
    trFractionalBits = list()
    trLineqAlpha = list()
    trLineqBeta = list()
    trLineqThreshold = list()

    def trackbarSigma(value):
        StereoConfigHandler.config.postProcessing.bilateralSigmaValue = value
        StereoConfigHandler.newConfig = True
        for tr in StereoConfigHandler.trSigma:
            tr.set(value)

    def trackbarConfidence(value):
        StereoConfigHandler.config.costMatching.confidenceThreshold = value
        StereoConfigHandler.newConfig = True
        for tr in StereoConfigHandler.trConfidence:
            tr.set(value)

    def trackbarLrCheckThreshold(value):
        StereoConfigHandler.config.algorithmControl.leftRightCheckThreshold = value
        StereoConfigHandler.newConfig = True
        for tr in StereoConfigHandler.trLrCheck:
            tr.set(value)

    def trackbarFractionalBits(value):
        StereoConfigHandler.config.algorithmControl.subpixelFractionalBits = value
        for tr in StereoConfigHandler.trFractionalBits:
            tr.set(value)

    def trackbarLineqAlpha(value):
        StereoConfigHandler.config.costMatching.linearEquationParameters.alpha = value
        StereoConfigHandler.newConfig = True
        for tr in StereoConfigHandler.trLineqAlpha:
            tr.set(value)

    def trackbarLineqBeta(value):
        StereoConfigHandler.config.costMatching.linearEquationParameters.beta = value
        StereoConfigHandler.newConfig = True
        for tr in StereoConfigHandler.trLineqBeta:
            tr.set(value)

    def trackbarLineqThreshold(value):
        StereoConfigHandler.config.costMatching.linearEquationParameters.threshold = value
        StereoConfigHandler.newConfig = True
        for tr in StereoConfigHandler.trLineqThreshold:
            tr.set(value)

    def handleKeypress(key, stereoDepthConfigInQueue):
        if key == ord('m'):
            StereoConfigHandler.newConfig = True
            medianSettings = [dai.MedianFilter.MEDIAN_OFF, dai.MedianFilter.KERNEL_3x3, dai.MedianFilter.KERNEL_5x5, dai.MedianFilter.KERNEL_7x7]
            currentMedian = StereoConfigHandler.config.postProcessing.median
            nextMedian = medianSettings[(medianSettings.index(currentMedian)+1) % len(medianSettings)]
            print(f"Changing median to {nextMedian.name} from {currentMedian.name}")
            StereoConfigHandler.config.postProcessing.median = nextMedian
        elif key == ord('c'):
            StereoConfigHandler.newConfig = True
            censusSettings = [dai.RawStereoDepthConfig.CensusTransform.KernelSize.AUTO, dai.RawStereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5, dai.RawStereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x7, dai.RawStereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x9]
            currentCensus = StereoConfigHandler.config.censusTransform.kernelSize
            nextCensus = censusSettings[(censusSettings.index(currentCensus)+1) % len(censusSettings)]
            print(f"Changing census transform to {nextCensus.name} from {currentCensus.name}")
            StereoConfigHandler.config.censusTransform.kernelSize = nextCensus
        elif key == ord('d'):
            StereoConfigHandler.newConfig = True
            dispRangeSettings = [dai.RawStereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64, dai.RawStereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_96]
            currentDispRange = StereoConfigHandler.config.costMatching.disparityWidth
            nextDispRange = dispRangeSettings[(dispRangeSettings.index(currentDispRange)+1) % len(dispRangeSettings)]
            print(f"Changing disparity range to {nextDispRange.name} from {currentDispRange.name}")
            StereoConfigHandler.config.costMatching.disparityWidth = nextDispRange
        elif key == ord('f'):
            StereoConfigHandler.newConfig = True
            StereoConfigHandler.config.costMatching.enableCompanding = not StereoConfigHandler.config.costMatching.enableCompanding
            state = "on" if StereoConfigHandler.config.costMatching.enableCompanding else "off"
            print(f"Companding {state}")
        elif key == ord('v'):
            StereoConfigHandler.newConfig = True
            StereoConfigHandler.config.censusTransform.enableMeanMode = not StereoConfigHandler.config.censusTransform.enableMeanMode
            state = "on" if StereoConfigHandler.config.censusTransform.enableMeanMode else "off"
            print(f"Census transform mean mode {state}")
        elif key == ord('1'):
            StereoConfigHandler.newConfig = True
            StereoConfigHandler.config.algorithmControl.enableLeftRightCheck = not StereoConfigHandler.config.algorithmControl.enableLeftRightCheck
            state = "on" if StereoConfigHandler.config.algorithmControl.enableLeftRightCheck else "off"
            print(f"LR-check {state}")
        elif key == ord('2'):
            StereoConfigHandler.newConfig = True
            StereoConfigHandler.config.algorithmControl.enableSubpixel = not StereoConfigHandler.config.algorithmControl.enableSubpixel
            state = "on" if StereoConfigHandler.config.algorithmControl.enableSubpixel else "off"
            print(f"Subpixel {state}")

        StereoConfigHandler.sendConfig(stereoDepthConfigInQueue)

    def sendConfig(stereoDepthConfigInQueue):
        if StereoConfigHandler.newConfig:
            StereoConfigHandler.newConfig = False
            configMessage = dai.StereoDepthConfig()
            configMessage.set(StereoConfigHandler.config)
            stereoDepthConfigInQueue.send(configMessage)

    def registerWindow(stream):
        cv2.namedWindow(stream)
        StereoConfigHandler.trConfidence.append(StereoConfigHandler.Trackbar('Disparity confidence', stream, 0, 255, StereoConfigHandler.config.costMatching.confidenceThreshold, StereoConfigHandler.trackbarConfidence))
        StereoConfigHandler.trSigma.append(StereoConfigHandler.Trackbar('Bilateral sigma', stream, 0, 100, StereoConfigHandler.config.postProcessing.bilateralSigmaValue, StereoConfigHandler.trackbarSigma))
        StereoConfigHandler.trLrCheck.append(StereoConfigHandler.Trackbar('LR-check threshold', stream, 0, 16, StereoConfigHandler.config.algorithmControl.leftRightCheckThreshold, StereoConfigHandler.trackbarLrCheckThreshold))
        StereoConfigHandler.trFractionalBits.append(StereoConfigHandler.Trackbar('Subpixel fractional bits', stream, 3, 5, StereoConfigHandler.config.algorithmControl.subpixelFractionalBits, StereoConfigHandler.trackbarFractionalBits))
        StereoConfigHandler.trLineqAlpha.append(StereoConfigHandler.Trackbar('Linear equation alpha', stream, 0, 15, StereoConfigHandler.config.costMatching.linearEquationParameters.alpha, StereoConfigHandler.trackbarLineqAlpha))
        StereoConfigHandler.trLineqBeta.append(StereoConfigHandler.Trackbar('Linear equation beta', stream, 0, 15, StereoConfigHandler.config.costMatching.linearEquationParameters.beta, StereoConfigHandler.trackbarLineqBeta))
        StereoConfigHandler.trLineqThreshold.append(StereoConfigHandler.Trackbar('Linear equation threshold', stream, 0, 255, StereoConfigHandler.config.costMatching.linearEquationParameters.threshold, StereoConfigHandler.trackbarLineqThreshold))

    def __init__(self, config):
        print("Control median filter using the 'm' key.")
        print("Control census transform kernel size using the 'c' key.")
        print("Control disparity search range using the 'd' key.")
        print("Control disparity companding using the 'f' key.")
        print("Control census transform mean mode using the 'v' key.")
        print("Control left-right check mode using the '1' key.")
        print("Control subpixel mode using the '2' key.")

        StereoConfigHandler.config = config


# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnBlobPath = sys.argv[1]

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.createColorCamera()
spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

xoutRgb = pipeline.createXLinkOut()
xoutNN = pipeline.createXLinkOut()
xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")


# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

camRgb.setFps(40)
monoLeft.setFps(40)
monoRight.setFps(40)

# Setting node configs
stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(False)
stereo.setSubpixel(False)

stereo.setRuntimeModeSwitch(True)
xinStereoDepthConfig = pipeline.createXLinkIn()
xinStereoDepthConfig.setStreamName("stereoDepthConfig")
xinStereoDepthConfig.out.link(stereo.inputConfig)
StereoConfigHandler(stereo.initialConfig.get())
StereoConfigHandler.registerWindow('depth')


spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    stereoDepthConfigInQueue = device.getInputQueue("stereoDepthConfig")

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    xoutBoundingBoxDepthMapping = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    startTime = time.monotonic()
    counter = 0
    fps = 0

    while True:
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame()

        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        detections = inDet.detections
        if len(detections) != 0:
            boundingBoxMapping = xoutBoundingBoxDepthMapping.get()
            roiDatas = boundingBoxMapping.getConfigData()

            for roiData in roiDatas:
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), 255, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        for detection in detections:
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
        cv2.imshow("depth", depthFrameColor)
        cv2.imshow("preview", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        StereoConfigHandler.handleKeypress(key, stereoDepthConfigInQueue)