#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
camRgb = pipeline.create(dai.node.ColorCamera)

xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutLeft.setStreamName('left')
xoutRight.setStreamName('right')
xoutVideo.setStreamName("video")

# Properties
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(800, 800)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
monoRight.out.link(xoutRight.input)
monoLeft.out.link(xoutLeft.input)
camRgb.video.link(xoutVideo.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames from the outputs defined above
    qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
    i =0 
    while True:
        # Instead of get (blocking), we use tryGet (nonblocking) which will return the available data or None otherwise
        inLeft = qLeft.tryGet()
        inRight = qRight.tryGet()
        videoIn = video.get()
        i = i+1 

        if inLeft is not None:
            left = cv2.resize(inLeft.getCvFrame(), (800,800))
            cv2.imshow("left", left)
            cv2.imwrite('Left'+str(i)+'.jpg',left)

        if inRight is not None:
            right = cv2.resize(inRight.getCvFrame(), (800,800))
            cv2.imshow("right", right)
            cv2.imwrite('Right'+str(i)+'.jpg',right)

        if videoIn is not None:
            rgb = videoIn.getCvFrame()
            cv2.imshow("RGB", rgb)
            cv2.imwrite('RGB'+str(i)+'.jpg', rgb)

            # Integral Image 
            # gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            imagecopy = cv2.copyMakeBorder(rgb, 1, 0, 1, 0, cv2.BORDER_CONSTANT, value=0)
            
            width = rgb.shape[0]
            height = rgb.shape[1]

#             for i in range(1, width):
#                 for j in range(1, height):
#                     imagecopy[i][j] = imagecopy[i - 1][j] + imagecopy[i][j - 1] - imagecopy[i - 1][j - 1] + rgb[i][j]
                    
            imagecopy = np.cumsum(imagecopy, axis=1).cumsum(axis=0)
            imagecopy = cv2.normalize(imagecopy, None, 255,0, cv2.NORM_MINMAX, cv2.CV_8UC1)
                   
            cv2.imshow("IMAGE", imagecopy)

        if cv2.waitKey(1) == ord('q'):
            break
