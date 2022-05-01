import cv2
import depthai as dai
import numpy as np
import datetime
import numpy as np
from datetime import datetime

# Setting variables 
initial_secs = 0
count = 0

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Properties 
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setVideoSize(1920, 1080)

# Setting the highest resolution 
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)

# Setting frames per second 
camRgb.setFps(70.0)
print('Getting FPS', camRgb.getFps())

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

camRgb.video.link(xoutVideo.input)


with dai.Device(pipeline) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    while True:
        videoIn = video.get()

        
        if videoIn is not None:
            frame = videoIn.getCvFrame()  
            
            if (initial_secs == 0):
                initial_secs = datetime.now().strftime('%M:%S.%f')[-9:-7]
                fps = count 
            
            secs = datetime.now().strftime('%M:%S.%f')[-9:-7]
            
            if (initial_secs == secs):
                count = count +1 

            else:
                initial_secs = secs
                if (count > fps):
                    fps = count
                count = 0
                print('Highest FPS:', fps)
                
            cv2.imshow("Stereo Pair", frame)
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
    
