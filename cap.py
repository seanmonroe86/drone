import cv2, time
import numpy as np
from threading import Thread

class ImageGetter:
    def __init__(self):
        self.currentFrame = None
        self.CAMERA_WIDTH = 640
        self.CAMERA_HEIGHT = 360
        self.CAMERA_NUM = 0
        self.capture = cv2.VideoCapture('tcp://192.168.1.1:5555')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.CAMERA_WIDTH)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.CAMERA_HEIGHT)

    def start(self):
        Thread(target=self.updateFrame, args=()).start()

    def updateFrame(self):
        while(True):
            ret, self.currentFrame = self.capture.read()
            while not ret: ret, frame = self.capture.read()

    def getFrame(self):
        return self.currentFrame

    def release(self):
        return self.capture.release()

cap = ImageGetter()
cap.start()
time.sleep(1)
try:
    while True:
        img = cap.getFrame()
        img = img[:, 185:455]
        img = cv2.resize(img, (120, 160))
        cv2.imshow("video", img)
        cv2.waitKey(1)
except KeyboardInterrupt:
    cap.release()

