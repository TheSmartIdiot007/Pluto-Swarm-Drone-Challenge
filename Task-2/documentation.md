# Documentation of control_node.py

## Class : control

### `__init__(self)`
Initializes the controller with the following attributes :
- `cap`: used to limit the outputs of the PID controller
- `log`: boolean to enable or disable the logging of the controller
- `deviation`: the deviation of Pluto from the true path
- `vision`: the vision object used to get the position of Pluto with respect to the camera
- `bridge`: the bridge object used to send and receive commands to and from Pluto
- `error_ready`: boolean which indicates when certain variables are ready to be logged
- `roll_output, pitch_output, throttle_output`: the outputs of the PID controller are initialized
- `delta_t`: the time between two iterations of the controller
- `prev_x, prev_y, prev_z`: the previous position of Pluto is initialized
- `integral_error_x, integral_error_y, integral_error_z`: the integral error of the PID controller is initialized
- `integral_saturation`: the integral term saturation error to prevent windup
- `Kp, Kd, Ki`: the PID gains are initialized in all the three axes (x, y, z)
- `slider_x, slider_y, slider_z`: the sliders used to tune the PID gains are initialized on multiple threads

### `loadPID_gains(self)`
Loads the PID gains from the `Xsaved.npy`, `Ysaved.npy` and `Zsaved.npy` files and sends them to the `update_K` method

### `update_K(self, val, name)`
Takes the value of the slider and the name of the slider as input and updates the corresponding PID gain, the gains are then divided by `self.delta_t` to compensate for the time between two iterations of the controller.

### `feedback(self)`
The feedback method takes the position feedback from the camera and updates the position of Pluto for the controller. 
- The output of the vision estimate is in centimeters, so it is converted to meters by dividing it by 100
- Since the camera is invertly mounted at a particular height, the position of Pluto is corrected by adding the height of the camera to the inverted z position of Pluto as obtained from the camera 
- After the relevant transformations, the position of Pluto is updated by calling the `update_position(self, current_position)` method

### `setpoint(self, desired_point, stay, time_interval, deadband_width)`
The inputs to this method are the following :
- `desired_point`: the desired position of Pluto
- `stay`: boolean to indicate whether Pluto should stay at the desired position or not
- `time_interval`: the time interval for which Pluto should stay at the desired position
- `deadband_width`: the distance from the desired position within which Pluto should be considered to be at the desired position

The desired position is fed into the `compute_control` method to compute the control inputs for the PID controller and then they are added to the zero values of throttle, roll and pitch to get the final control inputs. The control inputs are then sent to Pluto using the `bridge` object, by initializing variables `rcRoll, rcPitch, rcThrottle` and constraining them between tuned values for desired behaviour.

This process happens in a loop until the distance between the desired position and the current position is less than the deadband width or the time interval for `stay` is over.

### `compute_control(self, desired_x, desired_y, desired_z)`
The inputs to this method are the desired position of Pluto in the x, y and z axes. The method calls the `PID` method and returns updates the values of `roll_output, pitch_output` and `throttle_output`. The `PID` method is called for each of the three axes separately.

### `PID(self, final_position, current_position, Kp, Kd, Ki, integral_error, prev_position, max_saturation, min_saturation)`
This is the main method where the actual PID computations occur, the inputs to this method are the following :
- `final_position`: the desired position of Pluto
- `current_position`: the current position of Pluto
- `Kp, Kd, Ki`: the PID gains
- `integral_error`: the current value of the integral error of the PID controller
- `prev_position`: the previous position of Pluto
- `max_saturation, min_saturation`: the saturation limits for the integral error
  
The outputs of this function are the following :
- `pid_output`: the output of the PID controller
- `integral_error`: the updated value of the integral error
- `current_position`: the updated value of the current position of Pluto
- `position_error`: the current value of the error between the desired position and the current position

The error in x, y and z axes is fed to the controller and the PID control law is used to compute the values of the offset pitch, roll, and throttle (respectively) which are added to the corresponding zero values and sent to Pluto using the `bridge` object. 

For example, the error in x is fed to the controller which outputs `pitch_output` which is added to `zero_pitch_value` and then stored as `self.rcPitch`. Similarly, the error in y is fed to the controller which outputs `roll_output` which is added to `zero_roll_value` and then stored as `self.rcRoll`. And lastly, the error in z is fed to the controller which outputs `throttle_output` which is added to `zero_pitch_value` and then stored as `self.rcThrottle`.

### `clip(self, input, max, min)`
This method clips the given input between the maximum and minimum values, it is used to limit the values of `rcRoll, rcPitch, rcThrottle` to the maximum and minimum values that are feasible for Pluto. This method is also invoked during the integral error computation to prevent windup by limiting the integral error between the maximum and minimum saturation values.

## Class : task2

### `__init__(self, X_wp, Y_wp, altitude)`
Initializes the class with the following attributes :
- `X_wp, Y_wp`: the waypoint arrays in the x and y axes
- `altitude`: the altitude of Pluto with respect to the ground
- `x_density`: the density of the waypoints in the x axis (currently set to 4 points per meter)
- `y_density`: the density of the waypoints in the y axis (currently set to 3 points per meter)
- `controller`: the controller object used to control Pluto

### `create_rectangle(self, x_density, y_density)`
This method creates a rectangle of waypoints with the given density in the x and y axes. Since the desired rectangle is of 2 meters in the x axis and 1 meter in the y axis, there are 8 points in the x axis and 3 points in the y axis. The rectangle creation and the corresponding waypoint extraction is done using the `Rectangle` class.

### `track_rectangle(self)`
This method is used to track the rectangle of waypoints created by the `create_rectangle` method. The waypoints are extracted from the `Rectangle` class and then fed into the `controller` object to track the waypoints using the `setpoint` method.

### `__call__(self)`
This method is called when the `task2` object is called, it first sets the Pluto into developer mode, then arms it and then calls the `create_rectangle` method to create the rectangle of waypoints and then calls the `track_rectangle` method to track the waypoints. After the waypoints are successfully tracked, the Pluto is disarmed.

### `hover(self, z)`
This method uses bridge object to set developer mode and arm the drone followed by hovering at the specified altitude `z` 

# Documentation of slider.py
`tkinter` package is used to create a GUI based slider for real-time tuning of the PID gains. 

## Class: SliderInput

### `__init__(self, name, controller)`
Initializes the class with the following attributes :
- `name`: the name of the slider
- `controller`: the controller object used to control Pluto
- `sliderLength`: the length of the slider
- `range`: the range of the slider

The title and the dimensions of the slider window are decided using the variables `title`, `length` and `geometry`.

All the three sliders for the three axes x, y and z are initialized on different threads.

### `update(self)`
The value on the slider is read and is fed to the controller method `update_K` to update the PID gains. However the value obtained by the slider will be an integer and in order to obtain floating point values for the gains, the value is divided by 100. These values are then saved in the `Xsaved.npy`, `Ysaved.npy` and `Zsaved.npy` files using the `save_values` method.

### `prev_values(self)`
This method is used to load the values from the previous tuning session. The values are loaded from the `Xsaved.npy`, `Ysaved.npy` and `Zsaved.npy` files.

The `start_slider(name, controller)` method is used to start the slider for the given name and controller by initializing an object of the `SliderInput` class.

# Documentation of waypoint.py

## Class: Rectangle

### `__init__(self, X, Y, z, d0, d1)`
Initializes the class with the following attributes :
- `X, Y`: the waypoint arrays in the x and y axes
- `z`: the altitude of Pluto with respect to the ground
- `d0`: the density of the waypoints in the x axis (currently set to 4 points per meter)
- `d1`: the density of the waypoints in the y axis (currently set to 3 points per meter)

### `compute_slopes(self)`
This method computes the slopes of the lines connecting the waypoints in the x and y axes. 

### `generate_points(self)`
This method generates the points in the x and y axes using the `np.linspace` method.

### `deviation(self, i, cx, cy, cz)`
This method computes the deviation of the current position of Pluto from the line connecting the waypoints in the x and y axes. The inputs to this method are the following :
- `i`: the index of the point in the rectangle
- `cx, cy, cz`: the current position of Pluto


