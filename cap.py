import cv2, time
import numpy as np
from threading import Thread
from socket import socket, AF_INET, SOCK_STREAM

class VidSocket:
    def __init__(self):
        self.IP = "192.168.1.1"
        self.PORT = 5555
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((self.IP, self.PORT))

    def get(self):
        chunks = []
        bytes_recd = 0
        while bytes_recd < 2048:
            chunk = self.socket.recv(2048)
            if chunk == '':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd += len(chunk)
        return ''.join(chunks)

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
        cap = Thread(target=self.updateFrame, args=())
        cap.daemon = True
        cap.start()

    def updateFrame(self):
        while(True):
            ret, frame = self.capture.read()
            while not ret: ret, frame = self.capture.read()
            self.currentFrame = frame

    def getFrame(self):
        return self.currentFrame

    def release(self):
        return self.capture.release()

#cap = ImageGetter()
#cap.start()
cap = VidSocket()
print cap.get()
#time.sleep(1)
#try:
#    while True:
#        img = cap.getFrame()
#        #try:
#        #    test_img = img[:,:]
#        #except TypeError:
#        #    continue
#        cv2.imshow("video", img)
#        cv2.waitKey(1)
#except KeyboardInterrupt:
#    cap.release()

