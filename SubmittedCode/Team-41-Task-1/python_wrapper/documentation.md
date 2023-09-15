## Class Communication:

- A class for establishing and managing Telnet connections.
    
- Parameters:
    - `host (str)`: The hostname or IP address of the remote device.
    - `port (int)`: The port number to use for the Telnet connection.

- Attributes:
    - `host (str)`: The hostname or IP address of the remote device.
    - `port (int)`: The port number to use for the Telnet connection.
    - `socket (socket)`: The socket object representing the Telnet connection.
    - `socketList (list)`: A list of socket objects representing multiple Telnet connections.
    - `tnList (list)`: A list of Telnet objects representing multiple Telnet connections.
    - `timeout (float)`: The time to wait between write and read operations on the Telnet connection(s).

- Methods:
    - `connectSocket()`: Establish a single Telnet connection with the specified host and port.
    - `connectMultipleSocket(ip, index)`: Establish a Telnet connection with the specified IP address and port, and add it to the list of Telnet connections at the specified index.k
    - `writeSocket(data)`: Write data to the single Telnet connection.
    - `writeMultipleSocket(data, index)`: Write data to the Telnet connection at the specified index in the list of connections.
    - `readSocket()`: Read data from the single Telnet connection.
    - `readMultipleSocket(index)`: Read data from the Telnet connection at the specified index in the list of connections.

- Class MsgEncoder:
    - A class for encoding messages to be sent over a Telnet connection.
    
    - Attributes:
        - `HEADER (bytes)`: A list of bytes representing the header of the encoded message.
        - `DIRECTION (dict)`: A dictionary mapping message direction strings to corresponding bytes.
        - `MSP_MSG_PARSE (str)`: A format string for use with the `struct` module to pack the message for encoding.
        
    - Methods:
        - `encoder(msg, typeOfMsg)`: Encode a message with the specified type and return the encoded message as bytes.
        
    - Parameters:
        - `msg (list)`: A list of bytes representing the message to be encoded.
        - `typeOfMsg (int)`: An integer representing the type of the message.


## Class Pluto:

- A class representing a drone running the PlutoX firmware.
    
- Parameters:
    - `args`: An object containing arguments passed to the script at runtime.
    
- Attributes:
    - `args`: An object containing arguments passed to the script at runtime.
    - `lastTime (float)`: The time of the last update to the drone's state.
    - `pro (Protocol)`: An instance of the `Protocol` class for parsing data received from the drone.
    - `com (Communication)`: An instance of the `Communication` class for managing the Telnet connection to the drone.
    - `node (Thread)`: A thread for repeatedly requesting data from the drone.
    - `reader (Thread)`: A thread for continuously reading data from the drone.
    - `writer (Thread)`: A thread for continuously writing data to the drone.
    - `encoder (MsgEncoder)`: An instance of the `MsgEncoder` class for encoding data to be sent to the drone.
    - `MSP_ATTITUDE (bytes)`: The encoded message for requesting attitude data from the drone.
    - `MSP_ALTITUDE (bytes)`: The encoded message for requesting altitude data from the drone.
    - `MSP_RAW_IMU (bytes)`: The encoded message for requesting raw IMU data from the drone.
    - `avg_time (list)`: A list of average time intervals between data updates.
    - `armed (bool)`: A boolean indicating whether the drone is armed.
    
- Methods:
    - `writerFunc()`: Write RC control data to the drone.
    - `readerFunc()`: Continuously read data from the drone and update the `pro` attribute.
    - `_node()`: Continuously request data from the drone.
    - `reqData()`: Request attitude data from the drone.
    - `arm()`: Arm the drone.
    - `disarm()`: Disarm the drone.
    - `toggleArm()`: Toggle the armed state of the drone.
    - `send_data(roll, pitch, yaw, thrust)`: Send RC control data to the drone.
    - `altHold(hold = True)`: Set the altitude hold state of the drone.
    - `takeoff()`: Send a takeoff command to the drone.
    - `setDevMode()`: Set the drone's mode to developer mode.
    - `reset()`: Reset the drone's RC controls.
    - `setThrottle(throttle)`: Set the throttle value of the RC controls.



## Class Pose:
- A class representing a 3D position.
    
- Attributes:
    - `x (float)`: The x-coordinate of the position.
    - `y (float)`: The y-coordinate of the position.
    - `z (float)`: The z-coordinate of the position.
        
- Methods:
    - `__str__()`: Return a string representation of the object.

## Class acceleration(Pose):
- A class representing an acceleration in 3D space.
    
- Inherits from the `Pose` class.

## Class gyro(Pose):
- A class representing a gyroscope measurement in 3D space.
    
- Inherits from the `Pose` class.

## Class mag(Pose):
- A class representing a magnetometer measurement in 3D space.
    
- Inherits from the `Pose` class.


## Class Position:
- A class representing a position and orientation in 3D space.
    
- Attributes:
    - `pose (Pose)`: An instance of the `Pose` class representing the 3D position.
    - `orientation (Orientation)`: An instance of the `Orientation` class representing the orientation.



## Class rc:
- A class representing RC control data.
    
- Attributes:
    - `roll (int)`: The roll control value.
    - `pitch (int)`: The pitch control value.
    - `yaw (int)`: The yaw control value.
    - `throttle (int)`: The throttle control value.
    - `AUX1 (int)`: The value of the first auxiliary channel.
    - `AUX2 (int)`: The value of the second auxiliary channel.
    - `AUX3 (int)`: The value of the third auxiliary channel.
    - `AUX4 (int)`: The value of the fourth auxiliary channel.
    
- Methods:
    - `reset()`: Reset the RC control values to their default values.
    - `setArmVal()`: Set the RC control values for arming the drone.
    - `setDevMode()`: Set the RC control values for developer mode.
    - `update(roll, pitch, yaw, throttle)`: Update the RC control values.
    - `__call__()`: Return the RC control values as a list.
    - `__str__()`: Return a string representation of the object.


## Class Protocol:
- A class for parsing and storing data received from a drone running the PlutoX firmware.
    
- Attributes:
    - `data (bytes)`: The data received from the drone.
    - `position (Position)`: An object representing the position and orientation of the drone.
    - `acc (acceleration)`: An object representing the acceleration of the drone.
    - `gyro (gyro)`: An object representing the gyroscope readings of the drone.
    - `mag (mag)`: An object representing the magnetometer readings of the drone.
    - `alt (float)`: The altitude of the drone.
    - `roll (float)`: The roll angle of the drone.
    - `pitch (float)`: The pitch angle of the drone.
    - `yaw (float)`: The yaw angle of the drone.
    - `battery (float)`: The battery voltage of the drone.
    - `trim_roll (float)`: The roll trim value of the drone.
    - `trim_pitch (float)`: The pitch trim value of the drone.
    - `rc (rc)`: An object representing the RC control values of the drone.
    
- Methods:
    - `read16()`: Read and return a 16-bit integer from the data.
    - `read32()`: Read and return a 32-bit integer from the data.
    - `update()`: Parse and store the data received from the drone.
    -` __call__()`: Print the roll, pitch, and yaw angles and the current time.

