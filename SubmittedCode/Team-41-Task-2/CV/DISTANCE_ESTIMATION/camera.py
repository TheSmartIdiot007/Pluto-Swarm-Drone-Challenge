# import the opencv library
import cv2
import threading
import warnings
from time import time
import sys
sys.path.append("../../")
from CV.DISTANCE_ESTIMATION.pose_class import aruco_pose

from params import args  
# The Cam_feed class is for capturing and processing video frames from a camera
class Cam_feed:
    # The constructor for the class
    def __init__(self, args):
        print("Camera init")
        # Initialize the video capture object with the specified source (args.video_src)
        self.vid = cv2.VideoCapture(args.video_src)
        # Record the current time as the frame time
        self.frame_time = time()
        # Set the size of the aruco marker
        self.aruco_size = 5
        # Initialize the aruco pose object with the specified calibration file and aruco size
        self.pose = aruco_pose(args.calib_file,self.aruco_size)
        # The list of aruco marker IDs to track
        self.ids = [args.id_main]

        #set the width and height of the camera frame
        self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
        self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
        
        # set the video codec
        self.vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.vid.set(cv2.CAP_PROP_FPS, 60)
        # Initialize variables for the RGB image and its processed version
        self.rgb_img = None
        self.rgb_img_out = None
        # Initialize variables for the RGB image and its processed version
        self.X = {}
        self.Y = {}
        self.Z = {}
        self.camera_up = False
        self.log =  True

    # The update method updates the X, Y, and Z dictionaries with the latest distances and velocities
    def update(self):
        for id in self.ids:
            if id in self.X.keys():
                self.X[id] = [self.distance[id][0], self.velocity[id][0]]
                self.Y[id] = [self.distance[id][1], self.velocity[id][1]]
                self.Z[id] = [self.distance[id][2], self.velocity[id][2]]
            else:
                self.X.update({id:[self.distance[id][0], self.velocity[id][0]]})
                self.Y.update({id:[self.distance[id][1], self.velocity[id][1]]})
                self.Z.update({id:[self.distance[id][2], self.velocity[id][2]]})

    # The grab_frame method captures a frame from the camera and processes it to determine the distances and velocities of the aruco markers
    def grab_frame(self):
        self.t0 = time()
        ret, self.rgb_img = self.vid.read()
        t = time()
        # self.rgb_img = cv2.resize(self.rgb_img, (1280, 720))
        # self.rgb_img=self.rgb_img[100:1050,150:1570,:]
        # print(time()- t, "reshape time")
        # print(self.rgb_img.shape)
        self.frame_grabTime = time() - self.t0
        # print(1/self.frame_grabTime, 'frame grab')
        # self.update()
        t1 = time()
        
        try:

            if ret:
                self.distance, self.velocity, self.rgb_img_out = self.pose.pose1(self.rgb_img)
                
                try:
                    self.update()
                    self.camera_up = True
                except:
                    print('unable to process CV data', self.distance, self.t0)
            else:
                print("No frame captured", time())
        except:
            print("no aruco detected", time())
        # self.t1 = time()
        
        if self.log:
            self.frame_time = time() - self.t0
            print(1/self.frame_time, 'Frame time')

        print(1/(time() - self.t0), 'Frame time')
            

    #returns a string representation of the Cam_feed object
    def __str__(self) -> str:
        temp = ""
        for id in self.ids:
            temp += "id: {} ; X: {} Y: {} Z: {} yaw: {}, Vx: {} Vy: {} Vz: {} d_yaw: {}".format(id, self.X[id][0], self.Y[id][0], self.Z[id][0], self.yaw[id][0], self.X[id][1], self.Y[id][1], self.Z[id][1], self.yaw[id][1])
        return temp
                
# if __name__ == '__main__':
#     cam = Cam_feed(args)
#     cam.log = True
#     while True:
#         cam.grab_frame()
        # print(cam.distance)
        # print(cam)


