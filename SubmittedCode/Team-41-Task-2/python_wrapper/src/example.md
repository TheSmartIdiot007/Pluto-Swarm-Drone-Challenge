## Python Wrapper

### Initialization Phase:

-  The Pluto drone is turned on and the device is connected to its WiFi network named PLUTO_XXX. 
<!-- -  When keyboard control.py is executed, a KeyboardControl object is created that initialises the Pluto object and arms it. -->
- The Pluto object initialises the following:
    - The host and port arguments of the Pluto drone are passed to Pluto object in the cases of Tasks 1 and 2, and the IP address and port arguments of the Pluto drone are passed to Pluto object in the case of Task 3. This establishes communication between the device and drone through the connectSocket() method in the Communication object.
    - Threads for requesting data from drone and writing data to the drone are initialised.
    - MsgEncoder() object is initialized which will be used for encoding messages to be sent over a Telnet connection.
    - Signals of encoded message for requesting attitude data, altitude data, and raw IMU data from the drone are initialized.
<!-- - The key logger object and key listener thread are also initialized.
- The program waits for a key pressed by the user  -->

### Input Phase:
<!-- pointers ( key press from user, Contolkeyboard input to GetKey, Keyboard control object, writefunction )-->

- Each action has a corresponding function to calculate the desired output in the form of (RC_roll, RC_pitch, RC_yaw, RC_throttle) and it passes these values through the writerFunc function declared in the pluto class.

- Following are examples of functions to control the control:
    - The functions to send the altitude of the Pluto object
    ```python
    pluto.pro.rc.throttle = VALUE
    pluto.writerFunc()
    ```
    - The functions to send the yaw of the Pluto object
    ```python
    pluto.pro.rc.yaw = VALUE
    pluto.writerFunc()
    ```

    - The functions to send the pitch and roll of the Pluto object
    ```python
    pluto.pro.rc.pitch = VALUE
    pluto.writerFunc()

    pluto.pro.rc.roll = VALUE
    pluto.writerFunc()

    ```
    - The function to toggle the arm state of the drone. Arms or disarms the drone depending on its current state.
    ```python
    pluto.toggleArm()
    ```


### Communication and Actuation 
- The writerFunc in class Pluto uses writeSocket() fucntion in the Communication object to send roll, pitch, yaw and throttle data to the drone.
- The writeSocket() function sends data in bytes to the drone through socket.send(data) fucntion.
