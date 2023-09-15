import cv2 as cv
from cv2 import aruco
import numpy as np
import time
import numpy as np
from numpy.linalg import inv
import math



class aruco_pose:
    def __init__(self,calib_data_path,aruco_size):
        # Load the camera calibration data from the given path
        self.cam_mat,self.dist_coef,self.r_vectors,self.t_vectors = self.load_calib_data(calib_data_path)
        self.MARKER_SIZE = aruco_size # centimeters
        # Get the dictionary for 4X4_50 markers
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        # Create the parameters for marker detection
        self.param_markers = aruco.DetectorParameters_create()
        # Create Kalman filters for x, y, z and z1
        self.kf_x,self.kf_y,self.kf_z,self.kf_z1 = self.Kalman_Filter(),self.Kalman_Filter(),self.Kalman_Filter(),self.Kalman_Filter()
        self.prev_time=10
        self.curr_time=0
        self.prev_z=0
        self.iteration=0
        self.sat_dz = 500
        self.distance,self.velocity={},{}

    # Load the camera calibration data from the given path
    def load_calib_data(self,calib_data_path):
        calib_data = np.load(calib_data_path)
        cam_mat = calib_data["camMatrix"]
        dist_coef = calib_data["distCoef"]
        r_vectors = calib_data["rVector"]
        t_vectors = calib_data["tVector"]
        return cam_mat,dist_coef,r_vectors,t_vectors
        # print(calib_data.files)


    #Class for Kalman filter   
    class Kalman_Filter:
        def __init__(self):
                # Initialize time and state of the filter
                self.t0 = None
                self.filter_x = np.array([[0],[0]])
                self.filter_P = np.array([[5, 0],[0, 5]])
                self.filter_A = None
                self.filter_H = np.array([[1, 0]])
                self.filter_HT = np.array([[1],[0]])
                self.filter_R = 10
                self.filter_Q = np.array([[1, 0],[0, 3]])
        # Filter function to update the state and return the estimated values
        def filter(self,z):
            if self.t0 == None:
                # If the first iteration, set time and state transition matrix
                self.t0 = time.time()
                dt = 0.01
                self.filter_A = np.array([[1, dt],[0, 1]])
            else:
                # If not the first iteration, update the state transition matrix
                dt = time.time() - self.t0
                self.filter_A[0,1] = dt
                self.t0 = time.time()
            # Kalman filter equations
            x_p = self.filter_A@(self.filter_x)
            P_p = self.filter_A@(self.filter_P)@(np.transpose(self.filter_A)) + self.filter_Q
            S = self.filter_H@(P_p)@(self.filter_HT) + self.filter_R
            K = P_p@((self.filter_HT))*(1/S)
            residual = z - self.filter_H@(x_p) 
            self.filter_x = x_p + K*residual
            self.filter_P = P_p - K@(self.filter_H).dot(P_p)       
            return [self.filter_x[0], self.filter_x[1]]
            
    # Function to estimate the pose of the marker
    def pose1(self,frame, compute_frame = False, useKF = True):
        t3 = time.time()
        # Convert the frame to grayscale
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        t2 = time.time()
        # Detect the markers in the frame
        marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, self.marker_dict, parameters=self.param_markers)
        if marker_corners:
            t0 = time.time()
            # Estimate the pose of the markers
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef )
            t1 = time.time()
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                i = 0

                if useKF:
                    if tVec[i][0][2]-self.prev_z <self.sat_dz or self.iteration<100:
                        x,y,z = self.kf_x.filter(tVec[i][0][0]),self.kf_y.filter(tVec[i][0][1]),self.kf_z.filter(tVec[i][0][2])
                        z=self.kf_z1.filter(z[0])
                        distance,velocity = np.array([x[0][0],y[0][0],z[0][0]]),np.array([x[1][0],y[1][0],z[1][0]]) 
                        # print(distance, 'dist')                   
                    else:
                        distance,velocity=self.distance[ids[i]],self.velocity[ids[i]]
                    self.prev_z=distance[2]
                    
                    if ids[i] in self.distance.keys():
                        self.distance[ids[i]],self.velocity[ids[i]]=distance,velocity
                    else:
                        self.distance.update({ids[i]:distance})
                        self.velocity.update({ids[i]:velocity})
                else:
                    if tVec[i][0][2]-self.prev_z <self.sat_dz or self.iteration<100:
                        x, y, z = tVec[i][0][0], tVec[i][0][1], tVec[i][0][2]
                        distance,velocity = np.array([x ,y ,z ]),np.array([0, 0, 0])
                    else:
                        distance,velocity=self.distance[ids[i]],self.velocity[ids[i]]
                    self.prev_z=distance[2]

                    if ids[i] in self.distance.keys():
                        self.distance[ids[i]],self.velocity[ids[i]]=distance,velocity
                    else:
                        self.distance.update({ids[i]:distance})
                        self.velocity.update({ids[i]:velocity})
                # if compute_frame:
                #     # Draw the axes of the marker
                #     cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                #     corners = corners.reshape(4, 2)
                #     corners = corners.astype(int)
                #     top_left = corners[1].ravel()
                #     # Draw the ID of the marker
                #     cv.putText(frame,f"id: {ids[i]} Dist: {((distance))}",top_left,cv.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv.LINE_AA)
                #     point = cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
            self.prev_time=self.curr_time
            self.iteration=self.iteration+1
            t4 = time.time()
            # print (self.distance    )
            # print("cnvtcolor: {}, arucoDetecton: {}, estimation: {}, rest: {}".format(1/(t2-t3) , 1/(t0-t2) ,1/( t1-t0 ), 1/(t4-t1) ))
        else:
            print('No detection POSE CLASS', time.time())
        # print(self.distance, 'distance')
        
        return self.distance,self.velocity, frame       