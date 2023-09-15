## pose_class.py

- This script defines an `aruco_pose` class that can be used to determine the pose of a camera relative to a set of Aruco markers. 
The class has the following functions:

- `__init__(self, calib_data_path, aruco_size)`: This is the constructor for the class. It initializes the following instance variables:

    - `self.cam_mat`, `self.dist_coef`, `self.r_vectors`, `self.t_vectors`: These are matrices and vectors containing the camera calibration data, loaded from a file
      specified by `calib_data_path`.
    - `self.MARKER_SIZE`: The size of the Aruco markers in centimeters.
    - `self.marker_dict`: A dictionary of Aruco markers.
    - `self.param_markers`: Detector parameters for detecting Aruco markers.
    - `self.kf_r`, `self.kf_p`, `self.kf_yaw`, `self.kf_x`, `self.kf_y`, `self.kf_z`: Kalman filters for the pitch, roll, yaw, x, y, and z coordinates of the aruco's pose.
    - `self.prev_time`, `self.curr_time`: Timestamps for the previous and current frames.
    - `self.distance`, `self.velocity`, `self.theta`, `self.omega`: Dictionaries for storing the distance, velocity, orientation, and angular velocity of the each aruco        relative to camera.

- `load_calib_data(self, calib_data_path)`: This function loads the camera calibration data from a file specified by `calib_data_path` and returns it as a tuple of matrices and vectors.


- `Kalman_Filter`: This class defines a Kalman filter that can be used to smooth the pose data. It has the following functions:

    - `__init__(self)`: This is the constructor for the class. It initializes the following instance variables:

        - `self.filter_x`: The state vector of the filter.
        - `self.filter_P`: The state covariance matrix of the filter.
        - `self.filter_A`: The state transition matrix of the filter.
        - `self.filter_H`: The measurement matrix of the filter.
        - `self.filter_HT`: The transpose of the measurement matrix.
        - `self.filter_R`: The measurement noise covariance of the filter.
        - `self.filter_Q`: The process noise covariance of the filter.

    - `filter(self, z)`: This function updates the state of the Kalman filter with a new measurement z. It returns the filtered estimate of the state.

- `pose1(self,frame)`: This function processes a frame from the camera and computes the aruco's pose relative to the camera. It returns the distance, velocity, and orientation of the aruco, as well as an image of the frame with pose data overlaid on it.

## camera.py

- Class that captures real time video from a camera, processes the frames with a pose object, and stores the resulting pose data (distance, velocity, and 3D coordinates) in class variables. The Cam_feed class has the following functions:

- `__init__(self, args)`: This is the constructor for the class. It initializes the following instance variables:

    - `self.vid`: A cv2 video capture object that reads from the video source specified in args.video_src.
    - `self.frame_time:` The time at which the current frame was captured.
    - `self.aruco_size`: The size of the Aruco marker used to determine the pose of the camera.
    - `self.pose`: An aruco_pose object that processes frames and computes the aruco's pose relative to camera.
    - `self.rgb_img`: The most recent frame captured by the camera, in RGB format.
    - `self.rgb_img_out`: The same frame as `self.rgb_img`, but with pose data overlaid on top.
    - `self.X, self.Y, self.Z`: The 3D coordinates of the camera's pose.

- `grab_frame(self)`: This function captures a frame from the video stream and processes it with the pose object. It then stores the resulting pose data in the instance variables self.distance, self.velocity, self.rgb_img_out, self.X, self.Y, and self.Z. If no frame is captured, it prints "No frame captured".

- `update(self)`: A function that updates the X, Y, Z dictionaries with the latest position and velocity information.
- `_str__(self)`: A function that returns a string representation of the Cam_feed object, containing the position and velocity of each ArUco marker in the ids list.

