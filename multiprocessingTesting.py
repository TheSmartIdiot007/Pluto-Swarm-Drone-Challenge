import numpy as np
import multiprocessing as mp
from multiprocessing.managers import BaseManager
from time import time, sleep
from task_3.src.waypoint2 import Rectangle
import rospy
from std_msgs.msg import Float64

class a:
    def __init__(self, name):
        self.name = name
        self.hold = 0
        self.pos = np.array([0,0,0])
        self.desiredWP = np.array([0,0,0])
        self.n = 0

    def setpoint(self, wayPt, deadband_width= 0.1):
        self.desiredWP = wayPt
        self.n = 0
        while self.hold or self.n < 5:
            sleep(1)
            self.pos = self.pos + 0.1
            self.n += 1
            # print(self.name + " pos: ", self.pos)
class testClass:
    drone = []
    def __init__(self):
        # self.drone = []
        # self.drone.append(a('Drone1'))
        # self.drone.append(a('Drone2'))
        self.toggle_state = 0
        self.drone2WP = [[], [[-10, -10, -100], [10,10,100]], []] 
        self.X = np.array([-0.73, 0.8, 0.84, -.75])
        self.Y = np.array([0.45, 0.41, -0.31, -0.31])
        self.Z = 0.5 
        self.genPath()
        print(self.total)
        print(self.drone2WP)

    def get_drone(self):
        return self.drone


    def setup(self):
        self.drone.append(a('Drone1'))
        self.drone.append(a('Drone2'))
    def genPath(self):
        self.rect = Rectangle(self.X, self.Y, self.Z, 5, 4)
        self.rect.compute_slopes()
        rectWP = self.rect.generate_points()
        self.rectWP1 = rectWP[0:self.rect.S1,:]
        rectWP2 = rectWP[self.rect.S1:(self.rect.S1+self.rect.S2),:]
        rectWP3 = rectWP[(self.rect.S1+self.rect.S2):(self.rect.S1+self.rect.S2+self.rect.S3),:]
        rectWP4 = rectWP[(self.rect.S1+self.rect.S2+self.rect.S3):(self.rect.S1+self.rect.S2+self.rect.S3+self.rect.S4),:]
        self.total = [rectWP2,rectWP3,rectWP4,self.rectWP1]


    def targetDrone1(self):
        for waypts in self.total:
            for wayPt in waypts:
                self.drone[0].setpoint(wayPt)
                self.drone2WP[2].append(wayPt)
                print("waypt reached: Drone1", wayPt, self.toggle_state)
                sleep(1)
            self.toggle_state = 1
            self.drone2WP[0] = self.drone2WP[1]
            self.drone2WP[1] = self.drone2WP[2]
            self.drone2WP[2] = []
            print("toggle state: Drone1", self.toggle_state)
            self.drone[0].setpoint(waypts[-1])
        print("Drone 1 done")
        return
    def targetDrone2(self):
        self.drone[1].hold = 1
        while True:
            # print(self.toggle_state, self.drone2WP[0], 'from drone2')
            if len(self.drone2WP[0]) > 0:
                print(self.toggle_state, self.drone2WP[0], 'from drone2')
                for wayPt in self.drone2WP[0]:
                    self.drone[1].setpoint(wayPt)
                    print("waypt reached: Drone2", wayPt, self.toggle_state)
                    sleep(1)
                self.toggle_state = 1
                print("toggle state: Drone 2", self.toggle_state)
                self.drone[1].setpoint(self.drone2WP[0][-1])
            sleep(1.3)

    def toggle(self, idx):
        if self.drone[idx].hold == 1:
            self.drone[idx].hold = 0
        else:
            self.drone[idx].hold = 1

    def call(self):
        # while True:
        # print(self.toggle_state, 'from call')
        if self.toggle_state == 1:
            print('toggled by call')
            self.toggle_state = 0
            self.toggle(0)
            self.toggle(1)
        sleep(1)

class ROSLogger:
    def __init__(self, controller):
        self.controller = controller
        print('ROSlogger' + controller.name)
        self.pub = rospy.Publisher(self.controller.name + '/' +'Z_CV', Float64, queue_size=10)
    def call(self, controller):

        self.pub.publish(controller.pos[2])

if __name__ == '__main__':
    BaseManager.register('testClass', testClass)
    manager = BaseManager()
    manager.start()
    myClass = manager.testClass()
    myClass.setup()
    rospy.init_node('ROSlogger', anonymous=True)
    drone = myClass.get_drone()
    logger1 = ROSLogger(drone[0])
    logger2 = ROSLogger(drone[1])   
    print('initialization done')
    p1 = mp.Process(target=myClass.targetDrone1)
    p2 = mp.Process(target=myClass.targetDrone2)
    p1.start()
    p2.start()

    while True:
        myClass.call()
        drone = myClass.get_drone()
        logger1.call(drone)
        logger2.call(drone)


