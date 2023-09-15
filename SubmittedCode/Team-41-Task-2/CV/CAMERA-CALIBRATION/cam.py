import cv2 as cv
import numpy as np
from time import time
class Camera:
    def __init__(self,width=1920,height=1080):
        self.res_width=width
        self.res_height=height
        self.cap = cv.VideoCapture('/dev/video2')

        
        if self.res_width!=None and self.res_height!=None:
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.res_width)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.res_height)
            codec = cv.VideoWriter_fourcc(*'MJPG')
            self.cap.set(cv.CAP_PROP_FOURCC, codec)
            self.cap.set(cv.CAP_PROP_FPS, 60)
    def get_camera_frame(self):
        ret, frame = self.cap.read()
        print (frame.shape)
        return frame
    def end_camera_feed(self):
        self.cap.release()
        cv.destroyAllWindows()
Cam=Camera()
while True:
    t0 = time()
    frame=Cam.get_camera_frame()
    print(1/(time() - t0))
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
Cam.end_camera_feed()