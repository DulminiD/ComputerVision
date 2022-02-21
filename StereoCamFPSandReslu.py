import cv2
import depthai as dai
import numpy as np
from datetime import datetime


initial_secs = 0
count = 0


def getFrame(queue):
  frame = queue.get()
  return frame.getCvFrame()


def getMonoCamera(pipeline, isLeft):
    mono = pipeline.createMonoCamera()

  # Set Highest Camera Resolution
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    if isLeft:
      mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else :
      mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    return mono


def getStereoPair(pipeline, monoLeft, monoRight):
    
    stereo = pipeline.createStereoDepth()
    stereo.setLeftRightCheck(True)
    
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
  
    pipeline = dai.Pipeline()

    monoLeft = getMonoCamera(pipeline, isLeft = True)
    monoRight = getMonoCamera(pipeline, isLeft = False)

    # Setting highest FPS 
    monoLeft.setFps(120.0)
    monoRight.setFps(120.0)
    print(monoLeft.getFps())


    # Combine left and right cameras to form a stereo pair
    stereo = getStereoPair(pipeline, monoLeft, monoRight)
    
    xoutRectifiedLeft = pipeline.createXLinkOut()
    xoutRectifiedLeft.setStreamName("rectifiedLeft")

    xoutRectifiedRight = pipeline.createXLinkOut()
    xoutRectifiedRight.setStreamName("rectifiedRight")
    
    
    stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
    stereo.rectifiedRight.link(xoutRectifiedRight.input)
    

    with dai.Device(pipeline) as device:

        rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
        rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)


        cv2.namedWindow("Stereo Pair")
        cv2.setMouseCallback("Stereo Pair", mouseCallback)
        
        sideBySide = True

        while True:
            
            # Get left and right rectified frame
            leftFrame = getFrame(rectifiedLeftQueue);
            rightFrame = getFrame(rectifiedRightQueue)
            
            if sideBySide:
                # Show side by side view
                imOut = np.hstack((leftFrame, rightFrame))
            else :
                # Show overlapping frames
                imOut = np.uint8(leftFrame/2 + rightFrame/2)
                
            
            if (initial_secs == 0):
                initial_secs = datetime.now().strftime('%M:%S.%f')[-9:-7]
                fps = count
            
            secs = datetime.now().strftime('%M:%S.%f')[-9:-7]
            
            if (initial_secs == secs):
                count = count +1 
            else:
                initial_secs = secs
                if (count>0):
                    fps= count
                print('Highest FPS count upto now:', fps)
                count = 0
                
            
            imOut = cv2.cvtColor(imOut,cv2.COLOR_GRAY2RGB) 
            
            imOut = cv2.line(imOut, (mouseX, mouseY), (1280, mouseY), (0, 0, 255), 2)
            imOut = cv2.circle(imOut, (mouseX, mouseY), 2, (255, 255, 128), 2)
            cv2.imshow("Stereo Pair", imOut)
            

            
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('t'):
                sideBySide = not sideBySide
                

    cv2.destroyAllWindows()
