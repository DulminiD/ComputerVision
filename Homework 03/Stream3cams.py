import cv2
import depthai as dai
import numpy as np
from datetime import datetime



initial_secs = 0
count = 0
no =0

def getFrame(queue):
  frame = queue.get()
  return frame.getCvFrame()


def getMonoCamera(pipeline, isLeft):
    mono = pipeline.createMonoCamera()


    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    if isLeft:
      # Get left camera
      mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else :
      # Get right camera
      mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    return mono


def getStereoPair(pipeline, monoLeft, monoRight):
    # Configure stereo pair for depth estimation
    stereo = pipeline.createStereoDepth()
    # Checks occluded pixels and marks them as invalid
    stereo.setLeftRightCheck(True)
    
    # Configure left and right cameras to work as a stereo pair
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    return stereo

def mouseCallback(event,x,y,flags,param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX = x
        mouseY = y

if __name__ == '__main__':

    frameArray = []
    mouseX = 0
    mouseY = 640
    no = 1
    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    monoRight = getMonoCamera(pipeline, isLeft = False)
    monoLeft.setFps(120.0)
    monoRight.setFps(120.0)
    
    # Set up color cam 
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutVideo = pipeline.create(dai.node.XLinkOut)
    xoutVideo.setStreamName("video")

    # Color cam properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)

    # Set the resolution 
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setFps(60.0)
    
    xoutVideo.input.setBlocking(False)
    xoutVideo.input.setQueueSize(1)

    # Linking
    camRgb.video.link(xoutVideo.input)
    
    stereo = getStereoPair(pipeline, monoLeft, monoRight)

    
    xoutRectifiedLeft = pipeline.createXLinkOut()
    xoutRectifiedLeft.setStreamName("rectifiedLeft")

    xoutRectifiedRight = pipeline.createXLinkOut()
    xoutRectifiedRight.setStreamName("rectifiedRight")
    
    
    stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
    stereo.rectifiedRight.link(xoutRectifiedRight.input)
    
    
    # Pipeline is defined, now we can connect to the device

    with dai.Device(pipeline) as device:

        
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
        rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)
        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
        

        # Calculate a multiplier for colormapping disparity map

        cv2.namedWindow("Stereo Pair")
        cv2.setMouseCallback("Stereo Pair", mouseCallback)
        
        # Variable use to toggle between side by side view and one frame view.
        sideBySide = True

        while True:
            
            videoIn = video.get()
            
            if videoIn is not None:
                frame = videoIn.getCvFrame()
            
            start_time = datetime.now()
            
            # Get left and right rectified frame
            leftFrame = getFrame(rectifiedLeftQueue);
            rightFrame = getFrame(rectifiedRightQueue);
            
            if sideBySide:
                # Show side by side view
                imOut = np.hstack((leftFrame, rightFrame))
            else :
                # Show overlapping frames
                imOut = np.uint8(leftFrame/2 + rightFrame/2)
                
            imOut = cv2.cvtColor(imOut,cv2.COLOR_GRAY2RGB) 
            
            
            imOut = cv2.line(imOut, (mouseX, mouseY), (1280, mouseY), (0, 0, 255), 2)
            imOut = cv2.circle(imOut, (mouseX, mouseY), 2, (255, 255, 128), 2)
            rgbOut = cv2.resize(frame,(600,600))
            
            
            cv2.imshow("Result",imOut)
            cv2.imshow("Result",rgbOut)
            

            # Check for keyboard input
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('t'):
                sideBySide = not sideBySide
                
    cv2.destroyAllWindows()
