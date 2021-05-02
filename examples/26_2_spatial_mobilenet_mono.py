#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import matplotlib.pyplot as plt
import cmapy

'''
Mobilenet SSD device side decoding demo
  The "mobilenet-ssd" model is a Single-Shot multibox Detection (SSD) network intended
  to perform object detection. This model is implemented using the Caffe* framework.
  For details about this model, check out the repository <https://github.com/chuanqi305/MobileNet-SSD>.
'''

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

syncNN = True
showDisparity = 1  # Instead of passthrough depth (note: may not be properly synced)
camRes = dai.MonoCameraProperties.SensorResolution.THE_800_P # THE_720_P THE_400_P
nnFullCameraFov = 1
lrCheck = 0  # Note: rectified is no longer mirrored if this is enabled, we need to fix it in FW
extended = 0
subpixel = 0  # Warning: if enabled, 3D position will be wrong, as disparity and depth can't work together
disparityConfidenceThreshold = 230
if 0:  # OpenCV predefined maps
    colorMap = cv2.COLORMAP_JET  # COLORMAP_HOT, COLORMAP_TURBO, ...
else:  # matplotlib maps, see: https://matplotlib.org/stable/tutorials/colors/colormaps.html
    colorMap = cmapy.cmap('inferno')

flipRectified = False if lrCheck else True

# Get argument first
nnPath = str((Path(__file__).parent / Path('models/mobilenet-ssd_openvino_2021.2_6shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnPath = sys.argv[1]

if not Path(nnPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# Start defining a pipeline
pipeline = dai.Pipeline()


manip = pipeline.createImageManip()
manip.initialConfig.setResize(300, 300)
# The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
manip.setKeepAspectRatio(not nnFullCameraFov)

# Define a neural network that will make predictions based on the source frames
spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

manip.out.link(spatialDetectionNetwork.input)

# Create outputs
xoutManip = pipeline.createXLinkOut()
xoutManip.setStreamName("right")
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutManip.input)
else:
    manip.out.link(xoutManip.input)

depthRoiMap = pipeline.createXLinkOut()
depthRoiMap.setStreamName("boundingBoxDepthMapping")

xoutDepth = pipeline.createXLinkOut()
xoutDepth.setStreamName("depth")

nnOut = pipeline.createXLinkOut()
nnOut.setStreamName("detections")
spatialDetectionNetwork.out.link(nnOut.input)
spatialDetectionNetwork.boundingBoxMapping.link(depthRoiMap.input)

monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
monoLeft.setResolution(camRes)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(camRes)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
stereo.setLeftRightCheck(lrCheck)
stereo.setExtendedDisparity(extended)
stereo.setSubpixel(subpixel)
stereo.setConfidenceThreshold(disparityConfidenceThreshold)

stereo.rectifiedRight.link(manip.inputImage)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

if subpixel:
    # This is wrong (computed X,Y,Z), but allows the depth view (disparity actually) to be correct
    stereo.disparity.link(spatialDetectionNetwork.inputDepth)
else:
    stereo.depth.link(spatialDetectionNetwork.inputDepth)
if showDisparity:
    stereo.disparity.link(xoutDepth.input)
else:
    spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

# Connect and start the pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="right", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    depthRoiMap = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    rectifiedRight = None
    detections = []

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    while True:
        inRectified = previewQueue.get()
        det = detectionNNQueue.get()
        depth = depthQueue.get()

        counter += 1
        currentTime = time.monotonic()
        if (currentTime - startTime) > 1:
            fps = counter / (currentTime - startTime)
            counter = 0
            startTime = currentTime

        rectifiedRight = inRectified.getCvFrame()

        depthFrame = depth.getFrame()

        if showDisparity:
            depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, colorMap)
        else:
            maxDisparity = 95
            if extended: maxDisparity *= 2
            if subpixel: maxDisparity *= 32
            depthFrame = (depthFrame * 255. / maxDisparity).astype(np.uint8)
            depthFrameColor = cv2.applyColorMap(depthFrame, colorMap)
        detections = det.detections
        if len(detections) != 0:
            boundingBoxMapping = depthRoiMap.get()
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
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

        if flipRectified:
            rectifiedRight = cv2.flip(rectifiedRight, 1)

        # If the rectifiedRight is available, draw bounding boxes on it and show the rectifiedRight
        height = rectifiedRight.shape[0]
        width = rectifiedRight.shape[1]
        for detection in detections:
            if flipRectified:
                swap = detection.xmin
                detection.xmin = 1 - detection.xmax
                detection.xmax = 1 - swap
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)

            try:
                label = labelMap[detection.label]
            except:
                label = detection.label

            cv2.putText(rectifiedRight, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(rectifiedRight, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(rectifiedRight, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(rectifiedRight, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(rectifiedRight, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

            cv2.rectangle(rectifiedRight, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(rectifiedRight, "NN fps: {:.2f}".format(fps), (2, rectifiedRight.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        cv2.imshow("depth", depthFrameColor)
        cv2.imshow("rectified right", rectifiedRight)

        if cv2.waitKey(1) == ord('q'):
            break
