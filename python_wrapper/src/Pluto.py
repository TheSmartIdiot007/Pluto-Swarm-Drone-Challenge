import multiprocessing
import threading
import numpy as np
from time import sleep, time

import sys
sys.path.append("../../")
sys.path.append("../../python_wrapper/src/")
from params import args
from Protocol import Protocol
from Communication import Communication
from msgEncoder import MsgEncoder
from typeConst import *


class Pluto:
    '''Class representing a drone running the PlutoX firmware.
    
    Parameters:
        args: An object containing arguments passed to the script at runtime.
    
    Attributes:
        args: An object containing arguments passed to the script at runtime.
        lastTime (float): The time of the last update to the drone's state.
        pro (Protocol): An instance of the `Protocol` class for parsing data received from the drone.
        com (Communication): An instance of the `Communication` class for managing the Telnet connection to the drone.
        node (Thread): A thread for repeatedly requesting data from the drone.
        reader (Thread): A thread for continuously reading data from the drone.
        writer (Thread): A thread for continuously writing data to the drone.
        encoder (MsgEncoder): An instance of the `MsgEncoder` class for encoding data to be sent to the drone.
        MSP_ATTITUDE (bytes): The encoded message for requesting attitude data from the drone.
        MSP_ALTITUDE (bytes): The encoded message for requesting altitude data from the drone.
        MSP_RAW_IMU (bytes): The encoded message for requesting raw IMU data from the drone.
        avg_time (list): A list of average time intervals between data updates.
        armed (bool): A boolean indicating whether the drone is armed.
    
    Methods:
        writerFunc(): Write RC control data to the drone.
        readerFunc(): Continuously read data from the drone and update the `pro` attribute.
        _node(): Continuously request data from the drone.
        reqData(): Request attitude data from the drone.
        arm(): Arm the drone.
        disarm(): Disarm the drone.
        toggleArm(): Toggle the armed state of the drone.
        send_data(roll, pitch, yaw, thrust): Send RC control data to the drone.
        altHold(hold = True): Set the altitude hold state of the drone.
        takeoff(): Send a takeoff command to the drone.
        setDevMode(): Set the drone's mode to developer mode.
        reset(): Reset the drone's RC controls.
        setThrottle(throttle): Set the throttle value of the RC controls.
    '''

    def __init__(self, args):
        self.args = args
        self.lastTime = 0
        self.pro = Protocol()
        print('Task :', args.task)
        if args.task == 2 or args.task == 1:
            print('Connecting to ', self.args.host, ':', self.args.port, '...')
            self.com = Communication(self.args.host, self.args.port)
        elif args.task == 3:
            print('Connecting to ', self.args.ip, ':', self.args.port, '...')
            self.com = Communication(self.args.ip, self.args.port)

        self.node = threading.Thread(target=self._node)
        self.reader = threading.Thread(target=self.readerFunc)
        self.writer = threading.Thread(target=self.writerFunc)
        self.com.connectSocket()
        self.encoder = MsgEncoder()
        self.MSP_ATTITUDE = self.encoder.encoder([], MSP_ATTITUDE)
        self.MSP_ALTITUDE = self.encoder.encoder([], MSP_ALTITUDE)
        self.MSP_RAW_IMU = self.encoder.encoder([], MSP_RAW_IMU)  
        self.MSP_ANALOG= self.encoder.encoder([], MSP_ANALOG)      
        print(self.com.socket)
        self.node.start()
        self.reader.start()
        # self.writer.start()
        self.avg_time = []
        self.armed = False

    def writerFunc(self):
        # print(self.pro.rc)
        self.com.writeSocket(self.encoder.encoder(self.pro.rc(), MSP_SET_RAW_RC))

    def readerFunc(self):
        while True:
            self.t0 = time()
            self.pro.data = self.com.readSocket()
            self.pro.update()
            self.avg_time.append(1/(time() - self.t0))

    def _node(self):
        while True:
            self.lastTime = time()
            self.reqData()
            sleep(0.05)

    def reqData(self):
        self.com.writeSocket(self.MSP_ATTITUDE)
        self.com.writeSocket(self.MSP_ALTITUDE)
        self.com.writeSocket(self.MSP_ANALOG)

    def arm(self):
        print('arming')
        self.pro.rc.setArmVal()
        self.pro.rc.AUX4 = C
        self.writerFunc()
        self.armed = True


    def disarm(self):
        print('disarming')
        self.pro.rc.AUX4 = L
        self.writerFunc()
        self.armed = False

    def toggleArm(self):
        if self.armed:
            self.disarm()
        else:
            self.arm()

    def send_data(self, roll, pitch, yaw, thrust):
        self.pro.rc.AUX2 = C
        self.pro.update_RC(roll, pitch, yaw, thrust)
        # print(self.pro.rc)
        self.writerFunc()
    
    def altHold(self, hold = True):
        if hold:
            self.pro.rc.AUX3 = C
        else:
            self.pro.rc.AUX3 = L
        self.writerFunc()

    def takeoff(self):
        self.com.writeSocket(self.encoder.encoder([1], MSP_SET_COMMAND))

    def setDevMode(self):
        self.pro.rc.setDevMode()
        self.writerFunc()
    
    def reset(self, throttle = False):
        self.pro.rc.reset(self.args.rollZero, self.args.pitchZero, self.args.yawZero)
        if throttle:
            self.pro.rc.hardReset(self.args.rollZero, self.args.pitchZero, self.args.yawZero, self.args.throttleZero)
        self.writerFunc()

    def setThrottle(self, throttle):
        self.pro.rc.throttle = throttle
        self.writerFunc()

if __name__ == "__main__":
    args = args
    pluto = Pluto(args)
    pluto.toggleArm()
    sleep(2)

    
    pluto.setDevMode()
    t = 1700

    pluto.setThrottle(t)
    for i in range(100):
        pluto.setThrottle(t)
        print('throttle :' , t)
        sleep(.5)

    # sleep(2)
    pluto.setThrottle(1500)
    sleep(3)
    pluto.toggleArm()













    print(np.mean(pluto.avg_time))
    sleep(5)
    print(np.mean(pluto.avg_time))

    # while True:
    #     print("roll : {} pitch : {} yaw : {}".format(pluto.pro.roll, pluto.pro.pitch, pluto.pro.yaw))
    #     sleep(0.01)
    # # pluto.arm()

    # pluto.com.tn.close()
    # exit()
