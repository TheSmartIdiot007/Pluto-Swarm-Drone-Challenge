import numpy as np
from time import sleep, time
import sys
import threading
sys.path.append('../..')
from CV.DISTANCE_ESTIMATION.camera import Cam_feed
from python_wrapper.src.Pluto import Pluto
from slider2 import start_slider
from waypoint2 import Rectangle

class control :
    def __init__ (self, args) :
        self.args = args
        self.name = args.name
        # Initializing parameters for tuning and logging data
        self.cap = 30
        self.log = args.log
        self.deviation = 0.0
        self.rect = None
        self.hold = 0

        # Initializing an object of Cam_feed type for position feedback via camera
        # self.vision = Cam_feed(args)
        # self.vision.log =False

        # Initializing the communication bridge to send and receive data to and from (respectively) Pluto
        self.bridge = Pluto(args)

        # Boolean to start logging certain variables for efficient tuning
        self.error_ready = False
    
        # Initializing the PID outputs for roll, pitch and yaw
        self.roll_output = 0.0
        self.pitch_output = 0
        self.throttle_output = 0

        # Discretization time interval for the PID controller
        self.delta_t = 0.045 

        # Initializing the previous position of the drone
        self.prev_z = 0
        self.prev_x = 0
        self.prev_y = 0

        # Initialize the integral errors in x, y and z directions
        self.integral_error_x = 0
        self.integral_error_y = 0
        self.integral_error_z = 0

        # Set the integral saturation value to prevent integral windup
        self.integral_saturation = 150
        
        # Initialize position errors in x, y and z directions
        self.x_position_error = 0
        self.y_position_error = 0
        self.z_position_error = 0

        self.rcThrottle = 1500
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.distance = 0

        sleep(.1)

        # Initialize the gains for the controller
        self.Kp = np.array([4.0, 4.0, 5.0]) # proportional [x,y,z]
        self.Kd = np.array([1., 1., 8.5])  # derivative [x,y,z]
        self.Ki = np.array([0., 0., 0.1]) # integral [x,y,z]
        self.loadPID_gains()
        self.pos = [0,0,0]

        # Initialize the GUI based sliders for real-time PID tuning of Pluto
        # The initialization is done on multiple threads to ensure the real time effect of the all the three axes simulatenously
        self.slider_x = threading.Thread(target=start_slider, args=(self.name +'X', self))
        self.slider_y = threading.Thread(target=start_slider, args=(self.name +'Y', self))
        self.slider_z = threading.Thread(target=start_slider, args=(self.name +'Z', self))
        self.slider_x.start()
        self.slider_y.start()
        self.slider_z.start()
        # self.ROSlogger = ROSlogger(self)
        self.lastFeedbackTime = time()
        self.currentFeedbackTime = time()

    def loadPID_gains(self):
        self.update_K(np.load(self.name+'Xsaved.npy'), self.name +'X')
        self.update_K(np.load(self.name+'Ysaved.npy'), self.name +'Y')
        self.update_K(np.load(self.name+'Zsaved.npy'), self.name +'Z')


    # The update_K method reads the value of the gain from the slider and updates it for the controller
    def update_K(self, val, name):
        print("PID: {}, Kp: {}, Kd: {}, Ki: {}".format(name, val[0], val[1], val[2]))
        if name == self.name +'X':
            self.Kp[0] = val[0]
            self.Kd[0] = val[1]/self.delta_t
            self.Ki[0] = val[2]*self.delta_t
        elif name == self.name +'Y':
            self.Kp[1] = val[0]
            self.Kd[1] = val[1]/self.delta_t
            self.Ki[1] = val[2]*self.delta_t
        elif name == self.name +'Z':
            self.Kp[2] = val[0]
            self.Kd[2] = val[1]/self.delta_t
            self.Ki[2] = val[2]*self.delta_t


        
    # The update_position method takes the position feedback from the camera and stores it into a variable to be used by the controller
    def update_position(self, current_position) :
        # print(self.name, "current position: ", current_position)
        self.pos = current_position

    # The setpoint method calls the controller output and sends them to the Pluto drone
    def setpoint(self, desired_point, deadband_width = 0.1):
        print(self.name + " Desired point: ", desired_point, ' Current pose : ', self.pos)
        desired_point = np.reshape(desired_point, (3,))
        self.distance = np.linalg.norm([(self.pos - desired_point)[0], (self.pos - desired_point)[1]]) 
        self.dist_z = np.abs(self.pos[2] - desired_point[2])
        i = 0
        t_initial = None
        print(self.name, self.hold)
        while (self.distance >= deadband_width or self.hold or  self.dist_z >= .10) :
            # print(self.currentFeedbackTime, self.lastFeedbackTime, self.currentFeedbackTime > self.lastFeedbackTime)
            
            if self.currentFeedbackTime > self.lastFeedbackTime:
                self.lastFeedbackTime = self.currentFeedbackTime
                t0 = time()
                self.pos = np.reshape(self.pos, (3,))
                print(self.name, self.distance, self.hold, self.pos, desired_point)

                # print(self.name, self.pos, desired_point)

                self.compute_control(desired_point[0], desired_point[1], desired_point[2])
                
                zero_pitch_value = self.args.zero_pitch_value
                pitch_value = int(zero_pitch_value + 0.2*self.pitch_output) # x
                self.rcPitch= self.clip(pitch_value, zero_pitch_value + self.cap + 5, zero_pitch_value - self.cap)

            
                zero_roll_value = self.args.zero_roll_value
                roll_value = int(zero_roll_value + 0.4*self.roll_output) # y
                self.rcRoll = self.clip(roll_value, zero_roll_value + self.cap, zero_roll_value - self.cap)

                zero_throttle_value = self.args.zero_throttle_value
                throt_value = int(zero_throttle_value + self.throttle_output)
                self.rcThrottle = self.clip(throt_value, self.args.throttleMax, self.args.throttleMin)

                self.bridge.send_data(int(self.rcRoll), int(self.rcPitch), int(self.args.yaw_ff), int(self.rcThrottle))
                print(self.name, self.rcRoll, self.rcPitch, self.rcThrottle)
                
                if self.error_ready == False:
                    self.error_ready = True
                
                # self.deviation = self.rect.deviation(0, self.pos[0], self.pos[1], self.pos[2])s
                self.distance = np.linalg.norm([(self.pos - desired_point)[0], (self.pos - desired_point)[1]]) 
                self.dist_z = np.abs(self.pos[2] - desired_point[2])

                self.delta_t = time() - t0            

                if (self.distance < deadband_width) :
                    if i== 0:
                        t_initial = time()
                        i=i+1

                # self.ROSlogger()
                sleep(0.048)

                
        print ("Reached:", desired_point)

    # Calling the controller for x, y and z directions in compute_control method
    def compute_control(self, desired_x, desired_y, desired_z):
        self.roll_output, self.integral_error_y, self.prev_y, self.y_position_error = self.PID(desired_y, self.pos[1], self.Kp[1], self.Ki[1], self.Kd[1], self.integral_error_y, self.prev_y, self.integral_saturation, -1*self.integral_saturation)
        self.pitch_output, self.integral_error_x, self.prev_x, self.x_position_error = self.PID(desired_x, self.pos[0], self.Kp[0], self.Ki[0], self.Kd[0], self.integral_error_x, self.prev_x, self.integral_saturation, -1*self.integral_saturation)
        self.throttle_output, self.integral_error_z, self.prev_z, self.z_position_error = self.PID(desired_z, self.pos[2], self.Kp[2], self.Ki[2], self.Kd[2], self.integral_error_z, self.prev_z, self.integral_saturation, -1*self.integral_saturation)
    
    # The PID method does the control computation, by taking the current and the desired position
    # as an input and giving the corresponding command using the PID update law
    # The error in x is fed as an input and the output is the required pitch offset
    # The error in y is fed as an input and the output is the required roll offset
    # The error in z is fed as an input and the output is the required thrust offset
    def PID(self, final_position, current_position, K_p, K_i, K_d, integral_error, previous_position, max_saturation, min_saturation):
        position_error = final_position - current_position
        integral_error += position_error
        integral_error = self.clip(integral_error, max_saturation, min_saturation)
        derivative_error = current_position - previous_position
        pid_output = K_p*position_error + K_i*integral_error - K_d*derivative_error
        return pid_output, integral_error, current_position, position_error # current_position is passed as the previous_position in the next loop 

    # The clip method is used to constrain an input to specified boundaries
    def clip(self, input, max, min):
        if input > max:
            return max
        if input < min:
            return min
        else:
            return input
