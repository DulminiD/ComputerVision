#!/usr/bin/env python3

import cv2
import depthai as dai
import time 

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Propertiesq

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)
camRgb.setVideoSize(800, 800)

# Linking
camRgb.video.link(xoutVideo.input)
count = 0
imgs = []
test = []

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
    print('Usb speed: ', device.getUsbSpeed().name)
    stitchy=cv2.Stitcher.create()
    

    iterator = 10 

    while True:

        videoIn = video.get()
        image = videoIn.getCvFrame()
        image =cv2.resize(image,(0,0),fx=0.4,fy=0.4)

        if(iterator%15 == 0):
            imgs.append(image)
        
        cv2.imshow('Image', image)

        iterator = iterator + 1
        if cv2.waitKey(1) == ord('q'):
            break


    
(dummy,output)=stitchy.stitch(imgs)

if dummy != cv2.STITCHER_OK:
    print("stitching ain't successful")
else:
    print('Your Panorama is ready!!!')

if output.any():
    cv2.imshow('final result',output)

cv2.waitKey(0)
cv2.destroyAllWindows()