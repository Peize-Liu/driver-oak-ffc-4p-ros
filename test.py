#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)

xoutLeft.setStreamName('rgb')
xoutRight.setStreamName('camd')

# Properties
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_A)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_D)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)


# Linking
monoRight.out.link(xoutRight.input)
monoLeft.out.link(xoutLeft.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames from the outputs defined above
    qLeft = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="camd", maxSize=4, blocking=False)

    while True:
        # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        inLeft = qLeft.tryGet()
        inRight = qRight.tryGet()

        if inLeft is not None:
            cv2.imshow("rgb", inLeft.getCvFrame())

        if inRight is not None:
            cv2.imshow("camd", inRight.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break