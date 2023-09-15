import numpy as np
from time import sleep, time
import sys
import threading
sys.path.append('../..')
from CV.DISTANCE_ESTIMATION.camera import Cam_feed
from python_wrapper.src.Pluto import Pluto
from params import args
from param_1 import args1
from param_2 import args2
from param_3 import args3
from slider import start_slider
from waypoint import Rectangle
# from Debug_tool.ROSlogger import ROSlogger

class control :
    def __init__ (self, args) :
        # Initializing parameters for tuning and logging data
        self.name = args.name
        if self.name == 'Drone1':
            self.args = args1
        if self.name == 'Drone2':
            self.args = args2
        if self.name == 'Drone3':
            self.args = args3
        self.id = self.args.id_main
        self.cap = 20
        self.log = args.log
        self.deviation = 0.0
        self.rect = None
        self.pos = np.array([0,0,0])

        # Initializing an object of Cam_feed type for position feedback via camera
        self.vision = Cam_feed(self.args)
        self.vision.log =False

        # Initializing the communication bridge to send and receive data to and from (respectively) Pluto
        self.bridge = Pluto(self.args)

        # Boolean to start logging certain variables for efficient tuning
        self.error_ready = False
    
        # Initializing the PID outputs for roll, pitch and yaw
        self.roll_output = 0.0
        self.pitch_output = 0
        self.throttle_output = 0

        # Discretization time interval for the PID controller
        self.delta_t = 0.05 

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
        self.Cyaw = 0
        self.KpYaw = 0.02
        self.InitialYaw = []
        self.YawNotInit = True
        self.rcYaw = self.args.yaw_ff
        self.YawCorrection = 0

        sleep(.1)

        # Check whether vision is correctly working
        print('looking for vision')
        while not self.vision.camera_up:
            sleep(0.02)
            self.feedback()
        print('vision UP')

        # Initialize the gains for the controller
        self.Kp = np.array([4.0, 4.0, 5.0]) # proportional [x,y,z]
        self.Kd = np.array([1., 1., 8.5])  # derivative [x,y,z]
        self.Ki = np.array([0., 0., 0.1]) # integral [x,y,z]
        self.loadPID_gains()

        # Initialize the GUI based sliders for real-time PID tuning of Pluto
        # The initialization is done on multiple threads to ensure the real time effect of the all the three axes simulatenously
        self.slider_x = threading.Thread(target=start_slider, args=(self.name +'X', self))
        self.slider_y = threading.Thread(target=start_slider, args=(self.name +'Y', self))
        self.slider_z = threading.Thread(target=start_slider, args=(self.name +'Z', self))
        self.slider_x.start()
        self.slider_y.start()
        self.slider_z.start()
        # self.ROSlogger = ROSlogger(self)

    def loadPID_gains(self):
        self.update_K(np.load(self.name+'Xsaved.npy'), self.name+'X')
        self.update_K(np.load(self.name+'Ysaved.npy'), self.name+'Y')
        self.update_K(np.load(self.name+'Zsaved.npy'), self.name+'Z')


    # The update_K method reads the value of the gain from the slider and updates it for the controller
    def update_K(self, val, name):
        print("PID: {}, Kp: {}, Kd: {}, Ki: {}".format(name, val[0], val[1], val[2]))
        if name == self.name+'X':
            self.Kp[0] = val[0]
            self.Kd[0] = val[1]/self.delta_t
            self.Ki[0] = val[2]*self.delta_t
        elif name == self.name+'Y':
            self.Kp[1] = val[0]
            self.Kd[1] = val[1]/self.delta_t
            self.Ki[1] = val[2]*self.delta_t
        elif name == self.name+'Z':
            self.Kp[2] = val[0]
            self.Kd[2] = val[1]/self.delta_t
            self.Ki[2] = val[2]*self.delta_t

    # The feedback method takes the position of Pluto as a feedback from the camera using the vision object
    def feedback(self) :
        t0=time()

        self.vision.grab_frame()
        print (1/(time()-t0))
        self.Cyaw = self.bridge.pro.yaw
        if self.YawNotInit:
            if self.Cyaw is not None and len(self.InitialYaw) < 10:
                self.InitialYaw.append(self.Cyaw)
            elif self.YawNotInit:
                self.InitialYaw = np.average(self.InitialYaw)
                self.YawNotInit = False
        if self.id in self.vision.X.keys():

            # position feedback from CV
            feedback_x = self.vision.X[self.id][0] / 100
            feedback_y = self.vision.Y[self.id][0] / 100
            # feedback_z = 2.6132 -0.13 - (self.vision.Z[self.id][0]/100) # Transforming the z-coordinate of the camera to the ground frame
            feedback_z = 2.6132  -0.23 + 0.4 -.30- (self.vision.Z[self.id][0]/100) # Transforming the z-coordinate of the camera to the ground frame
    
            self.feedback_pos = np.array([feedback_x, feedback_y, feedback_z])
            self.update_position(self.feedback_pos) # updating the position of the drone for the controller 

        
    # The update_position method takes the position feedback from the camera and stores it into a variable to be used by the controller
    def update_position(self, current_position) :
        self.pos = current_position

    # The setpoint method calls the controller output and sends them to the Pluto drone
    def setpoint(self, desired_point, stay=False, time_interval = 10, deadband_width = 0.1):
        desired_point = np.reshape(desired_point, (3,))
        self.pos = np.reshape(self.pos, (3,))
        self.distance = np.linalg.norm([(self.pos - desired_point)[0], (self.pos - desired_point)[1]]) 
        self.distance_z = abs((self.pos - desired_point)[2])
        print(self.distance, self.distance_z)
        deadband_width_z = 0.10
        i = 0
        t_initial = None
        while (((self.distance >= deadband_width) or (self.distance_z >= deadband_width_z)) or stay) :
            t0 = time()
            self.feedback()
            self.pos = np.reshape(self.pos, (3,))
            self.compute_control(desired_point[0], desired_point[1], desired_point[2])
            
            zero_pitch_value = self.args.zero_pitch_value
            pitch_value = int(zero_pitch_value + 0.2*self.pitch_output) # x
            self.rcPitch= self.clip(pitch_value, zero_pitch_value + self.cap + 5, zero_pitch_value - self.cap)

        
            zero_roll_value = self.args.zero_roll_value
            roll_value = int(zero_roll_value + 0.2*self.roll_output) # y
            self.rcRoll = self.clip(roll_value, zero_roll_value + self.cap + 45 , zero_roll_value - self.cap - 10)

            zero_throttle_value = self.args.zero_throttle_value
            throt_value = int(zero_throttle_value + self.throttle_output)
            self.rcThrottle = self.clip(throt_value, self.args.throttleMax, self.args.throttleMin)
            # print(self.rcThrottle)

            if self.YawNotInit == False:
                yawError = self.InitialYaw - self.Cyaw
                self.YawCorrection = self.KpYaw*yawError
            self.rcYaw -= int((200/360)*self.YawCorrection)
            self.rcYaw = self.clip(self.rcYaw, 1550, 1450)


            self.bridge.send_data(int(self.rcRoll), int(self.rcPitch), int(self.rcYaw), int(self.rcThrottle))
            # print(self.rcYaw)
            
            if self.error_ready == False:
                self.error_ready = True
            
            self.deviation = self.rect.deviation(0, self.pos[0], self.pos[1], self.pos[2])
            self.distance = np.linalg.norm([(self.pos - desired_point)[0], (self.pos - desired_point)[1]]) 
            self.distance_z = abs((self.pos - desired_point)[2])
            print(self.pos, desired_point, self.distance)

            self.delta_t = time() - t0            

            if (self.distance < deadband_width) :
                if i== 0:
                    t_initial = time()
                    i=i+1

            if t_initial is not None :
                if time() - t_initial > time_interval :
                    stay = False

            # self.ROSlogger()
                
        print ("Reached:", desired_point)

    # Calling the controller for x, y and z directions in compute_control method
    def compute_control(self, desired_x, desired_y, desired_z):
        self.roll_output, self.integral_error_y, self.prev_y, self.y_position_error = self.PID(desired_y, self.pos[1], self.Kp[1], self.Ki[1], self.Kd[1], self.integral_error_y, self.prev_y, self.integral_saturation, -1*self.integral_saturation)
        self.pitch_output, self.integral_error_x, self.prev_x, self.x_position_error = self.PID(desired_x, self.pos[0], self.Kp[0], self.Ki[0], self.Kd[0], self.integral_error_x, self.prev_x, self.integral_saturation, -1*self.integral_saturation)
        self.throttle_output, self.integral_error_z, self.prev_z, self.z_position_error = self.PID(desired_z, self.pos[2], self.Kp[2], self.Ki[2], self.Kd[2], self.integral_error_z, self.prev_z, self.integral_saturation, -0.5*self.integral_saturation)
    
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

class task2: 
    def __init__(self, X_wp, Y_wp, altitude) :
        self.Xwp = X_wp
        self.Ywp = Y_wp
        self.altitude = altitude
        self.x_density = 4
        self.y_density = 3
        self.controller = control(args=args)
        self.create_rectangle(self.x_density, self.y_density)
        
        
    # create_rectangle method takes the desired input points per unit metre as x_density and y_density
    def create_rectangle(self, x_density, y_density) :
        self.rect = Rectangle(self.Xwp, self.Ywp, self.altitude, x_density, y_density)
        self.rect.compute_slopes()
        self.rectWP = self.rect.generate_points()
        self.controller.rect = self.rect
    
    def track_rectangle(self) :
        self.controller.setpoint(self.rectWP[0], stay=True, time_interval=8, deadband_width=0.15)
        for wp in self.rectWP :
            self.controller.setpoint(wp, stay=False, deadband_width=0.15)
        
        land_position = self.rectWP[-1]
        land_position[-1] = 0.1
        self.controller.setpoint(land_position, stay=True, time_interval=0.5, deadband_width=0.2)

    def __call__(self) :
        self.controller.bridge.setDevMode()
        self.controller.bridge.arm()
        sleep(2)
        
        self.track_rectangle()

        self.controller.bridge.disarm()

    def hover(self, z):
        self.controller.bridge.setDevMode()
        self.controller.bridge.arm()
        sleep(2)
        self.controller.setpoint([0,0,z], stay=True, time_interval= 120, deadband_width=.05)
        self.controller.bridge.disarm()
        

if __name__ == '__main__' :
    # X = np.array([0.66, -1.03, -1.1, 0.61])
    # Y = np.array([-0.42, -0.58, 0.28, 0.43])
    # X = np.array([0.72, 0.8, -0.93, -1.03])
    # Y = np.array([0.42, -0.43, -0.58, 0.28])
    X = np.array([0.4, -1.1, -1.1, 0.3])
    Y = np.array([0.3, 0.33, -0.6, -0.5])
    Z = 0.4
    hover = False
    T2 = task2(X, Y, Z)
    sleep(4)
    if hover:
        T2.hover(Z)
    else:
        T2()

    sys.exit()
