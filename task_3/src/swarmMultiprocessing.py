import numpy as np
import math
from time import sleep, time
from control_node import control


import sys
import multiprocessing as mp
from multiprocessing.managers import BaseManager
sys.path.append('../..')
sys.path.append('../')
from waypoint2 import Rectangle
from param_1 import args1
from param_2 import args2
from param_3 import args3
from params import args
import sys
import threading
sys.path.append('../..')
from CV.DISTANCE_ESTIMATION.camera import Cam_feed
# import rospy
# from ROSLoggerMultiprocessing import ROSlogger

# ids = [args1.id_main, args2.id_main]
class Swarm:
    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z
        self.drone1 = args1
        self.drone2 = args3
        self.drone = [control(self.drone1), control(self.drone2)]
        # self.logger1 = ROSlogger(self.drone[0])
        # self.logger2 = ROSlogger(self.drone[1])
        self.record_wp = np.array([])
        self.toggle_count = 0
        self.vision = Cam_feed(args)
        self.vision.log =False
        self.genPath()
        self.moveDrone = [1, 0] 
        self.wayPtDrone2 = [ [], self.rectWP1, np.array([])]
        self.runDone = False
        self.runDoneDrone2 = False
        self.toggle_state = 0
        self.arm()
        # self.drone[0].bridge.arm()
        # self.t1 = threading.Thread(target = self.TargetFnDrone1)
        # self.t2 = threading.Thread(target = self.TargetFnDrone2)
        # self.t1.start()
        # self.t2.start()
    def getDrone(self):
        return self.drone

    def arm(self):
        for drone in self.drone:
            drone.bridge.arm()
            sleep(1)
        
    def feedback(self, print_feedback=False) :
        t0 = time()
        # print(self.vision.X)
        for id in self.vision.X.keys():
            feedback_x = self.vision.X[id][0] / 100
            feedback_y = self.vision.Y[id][0] / 100
            self.vision_z = 2.6132  -0.23 - (self.vision.Z[id][0]/100)
            feedback_z = self.vision_z
            self.feedback_pos = np.array([feedback_x, feedback_y, feedback_z])

            feedback_vx = self.vision.X[id][1] / 100
            feedback_vy = self.vision.Y[id][1] / 100
            feedback_vz = self.vision.Z[id][1] / 100
            self.feedback_vel = np.array([feedback_vx, feedback_vy, feedback_vz])
            
            if print_feedback :
                print ("feedback pos:", self.feedback_pos)
                print ("feedback vel:", self.feedback_vel)
            if id == self.drone1.id_main:
                self.drone[0].update_position(self.feedback_pos)
            elif id == self.drone2.id_main:
                self.drone[1].update_position(self.feedback_pos)

    def genPath(self):
        self.rect = Rectangle(self.X, self.Y, self.Z, 5, 4)
        self.rect.compute_slopes()
        rectWP = self.rect.generate_points()
        self.rectWP1 = rectWP[0:self.rect.S1,:]
        rectWP2 = rectWP[self.rect.S1:(self.rect.S1+self.rect.S2),:]
        rectWP3 = rectWP[(self.rect.S1+self.rect.S2):(self.rect.S1+self.rect.S2+self.rect.S3),:]
        rectWP4 = rectWP[(self.rect.S1+self.rect.S2+self.rect.S3):(self.rect.S1+self.rect.S2+self.rect.S3+self.rect.S4),:]
        self.total = [rectWP2,rectWP3,rectWP4,self.rectWP1]

    def TargetFnDrone1(self):
        print('Called Drone 1')
        for wayPts in self.total:
            lastwayPt = wayPts[-1]
            for wayPt in wayPts:
                self.drone[0].setpoint(wayPt, deadband_width= 0.1)
                if len(self.wayPtDrone2[2]) < 1:
                    self.wayPtDrone2[2] = (self.drone[0].pos).reshape((1,3))
                else:
                    self.wayPtDrone2[2] = np.vstack([self.wayPtDrone2[2], np.reshape([self.drone[0].pos[0],self.drone[0].pos[1],self.Z],(1,3))])
            self.wayPtDrone2[0] = self.wayPtDrone2[1]
            self.wayPtDrone2[1] = self.wayPtDrone2[2]
            self.wayPtDrone2[2] = np.array([])
            self.drone[0].hold = 1
            self.moveDrone = [0,1]
            self.toggle_state = 1
            self.drone[0].setpoint(lastwayPt, deadband_width= 0.1)
        self.drone[0].bridge.disarm()
        self.runDone = True



    def TargetFnDrone2(self):
        print('Called Drone 2')
        while True:
            sleep(2)
            if len(self.wayPtDrone2[0]) == 0:
                sleep(1)
            elif self.moveDrone[1]==1:
                print('Desired pts drone 2',self.wayPtDrone2[0])
                for wayPt in self.wayPtDrone2[0]:
                    self.drone[1].setpoint(wayPt, deadband_width= 0.1)
                self.drone[1].hold = 1
                self.moveDrone = [1,0]
                self.toggle_state = 1
                self.drone[1].setpoint(wayPt, deadband_width= 0.1)
            if self.runDoneDrone2 == True:
                self.drone[1].bridge.disarm()
                return
    def updateFeedbackTime(self):
        self.drone[0].currentFeedbackTime = time()
        self.drone[1].currentFeedbackTime = time()
    def call(self):
        t0 = time()
        self.vision.grab_frame()
        
        self.feedback()
        self.updateFeedbackTime()
        # self.logger1()
        # self.logger2()
        if self.toggle_state:
            if self.moveDrone[0] == 0:
                self.drone[1].hold = 0
                self.toggle_state = 0
                print('Drone 2 toggled')
            if self.moveDrone[1] == 0:
                self.drone[0].hold = 0
                self.toggle_state = 0
                print('Drone 1 toggled')
        if self.runDone == True:
            self.drone[1].hold = 0
            self.runDoneDrone2 = True
            return False
        print('fps ', 1/(time()-t0))
        return True


              

if __name__ == '__main__' :
    # X = np.array([-0.57, 1.1, 1.1, -0.55])
    # Y = np.array([0.55, 0.57, -0.45, -0.45])
    # Z = 0.4
    X = np.array([-0.6, 0.9, 0.9, -0.6])
    Y = np.array([-0.25, -0.25, 0.63, 0.6])
    Z = 0.4
    # swarm = Swarm(X, Y, Z)
    # swarm()
    # rospy.init_node('ROSlogger', anonymous=True)
    
    BaseManager.register('Swarm', Swarm)
    manager = BaseManager()
    manager.start()
    myClass = manager.Swarm(X, Y, Z)
    # drone = myClass.getDrone()
    # logger = ROSlogger(drone[0])
    # logger1 = ROSlogger(drone[0])
    # logger2 = ROSlogger(drone[1]) 
    print('initialization done')
    p1 = mp.Process(target=myClass.TargetFnDrone1)
    p2 = mp.Process(target=myClass.TargetFnDrone2)
    p1.start()
    p2.start()
    while myClass.call():
        # drone = myClass.getDrone()
        # logger1(drone[0])
        # logger2(drone[1])
        pass
    p1.join()

    
