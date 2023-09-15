from Position import Position, acceleration, gyro, mag
from time import sleep
from copy import deepcopy
from typeConst import *
import numpy as np
import time

class rc:
    def __init__(self):
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1000
        self.throttle = 1000

        self.AUX1 = C
        self.AUX2 = C
        self.AUX3 = C
        self.AUX4 = 1200
    
    def reset(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        # self.throttle = 1500
    
    def hardReset(self, roll, pitch, yaw, throttle):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.throttle = throttle

    def setArmVal(self):
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = 1000

    def setDevMode(self):
        self.AUX1 = L
        self.AUX2 = C
        self.AUX3 = H
        self.AUX4 = C
        self.yaw = 1500
    
    def update(self, roll, pitch, yaw, throttle):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.throttle = throttle

    def __call__(self):
        return [self.roll, self.pitch, self.throttle, self.yaw, self.AUX1, self.AUX2, self.AUX3, self.AUX4]

    def __str__(self):
        return "roll: {}, pitch: {}, yaw: {}, throttle: {}".format(self.roll, self.pitch, self.yaw, self.throttle)

class Kalman_Filter:
    def __init__(self):
            self.t0 = None
            self.filter_x = np.array([[0],[0]])
            self.filter_P = np.array([[5, 0],[0, 5]])
            self.filter_A = None
            self.filter_H = np.array([[1, 0]])
            self.filter_HT = np.array([[1],[0]])
            self.filter_R = 10
            self.filter_Q = np.array([[1, 0],[0, 3]])

    def filter(self,z):
        if self.t0 == None:
            self.t0 = time.time()
            dt = 0.01
            self.filter_A = np.array([[1, dt],[0, 1]])
        else:
            dt = time.time() - self.t0
            self.filter_A = np.array([[1, dt],[0, 1]])
            self.t0 = time.time()
        x_p = self.filter_A@(self.filter_x)
        P_p = self.filter_A@(self.filter_P)@(np.transpose(self.filter_A)) + self.filter_Q
        S = self.filter_H@(P_p)@(self.filter_HT) + self.filter_R
        K = P_p@((self.filter_HT))*(1/S)
        residual = z - self.filter_H@(x_p) 
        self.filter_x = x_p + K*residual
        self.filter_P = P_p - K@(self.filter_H).dot(P_p)       
        return [self.filter_x[0], self.filter_x[1]]

class Protocol:
    def __init__(self):
        self.data = None
        self.position = Position()
        self.acc = acceleration()
        self.gyro = gyro()
        self.gyro_KF = gyro()
        self.mag = mag()
        self.alt = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = None
        self.battery = 0.0
        self.rssi = 0.0
        self.alt_KF_out = 0.0
        self.roll_KF = Kalman_Filter()
        self.pitch_KF = Kalman_Filter()
        self.yaw_KF = Kalman_Filter()

        self.alt_KF = Kalman_Filter()

        self.FC_versionMajor = 0.
        self.FC_versionMinor = 0.
        self.FC_versionPatch = 0.

        self.trim_roll = 0.
        self.trim_pitch = 0.
        self.battery = 0.0

        self.rc = rc()

    def __str__(self):
        return "roll : {}, pitch : {}, yaw : {}, time {}".format(self.gyro_KF.x, self.gyro_KF.y, self.gyro_KF.z, time.time())

    def read16(self):
        temp = deepcopy(self.msg[:2])
        self.msg = self.msg[2:]
        return np.frombuffer(temp, dtype=np.int16)[0]

    def read32(self):
        temp = deepcopy(self.msg[:4])
        self.msg = self.msg[4:]
        return np.frombuffer(temp, dtype=np.int32)
    def read8(self):
        temp = deepcopy(self.msg[:1])
        self.msg = self.msg[1:]
        return np.frombuffer(temp, dtype=np.int8)
    
    def update_rates(self):
        self.gyro_KF.x = self.roll_KF.filter(self.roll)
        self.gyro_KF.y = self.pitch_KF.filter(self.pitch)
        self.gyro_KF.z = self.yaw_KF.filter(self.yaw)

    def update_RC(self, roll, pitch, yaw, throttle):
        self.rc.roll = roll
        self.rc.pitch = pitch
        self.rc.yaw = yaw
        self.rc.throttle = throttle

    def update(self):
        if len(self.data) > 4:
            command = self.data[4]
            self.msg = deepcopy(self.data[5:])
            if self.data[3] +1 == len(self.msg):
                if command == MSP_ALTITUDE:
                    self.alt = self.read32()
                    self.alt_KF_out = self.alt_KF.filter(self.alt)
                
                elif command == MSP_ATTITUDE:
                    self.roll = self.read16()
                    self.pitch = self.read16()
                    self.yaw = self.read16()
                    self.update_rates()

                elif command == MSP_RAW_IMU:
                    self.acc.x = self.read16()
                    self.acc.y = self.read16()
                    self.acc.z = self.read16()

                    self.gyro.x = self.read16()
                    self.gyro.y = self.read16()
                    self.gyro.z = self.read16()

                    self.mag.x = self.read16()
                    self.mag.y = self.read16()
                    self.mag.z = self.read16()
                
                elif command == MSP_ANALOG:
                    self.battery = self.read8()/10.0
        # print(self)