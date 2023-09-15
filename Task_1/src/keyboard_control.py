from key_command import GetKey
import threading
from time import time, sleep

import sys

sys.path.append("../../")

from python_wrapper.src.Pluto import Pluto
from params import args

# This class defines a keyboard controller for a Pluto object
class KeyboardControl:
    def __init__(self, args):
        # Initialize the Pluto object and arm it
        self.pluto = Pluto(args)
        self.pluto.arm()
        sleep(2)
        
        # Initialize the key logger object and key listener thread
        self.key_logger = GetKey()
        self.key_listener = threading.Thread(target=self._key_listener)
        self.key_listener.start()
        
        # Set the time of the last key press to the current time
        self.last_key_strock = time()
        
        # Set the reset flag to False
        self.reset_called = False

        # Set the timeouts for the main loop
        self.timeouts = 0.01
        
        # Create a dictionary mapping keys to functions
        self.keyMap = {'w': self.incAlt,
                       's': self.decAlt,
                       'a': self.yawLeft,
                       'd': self.yawRight,
                       'q': self.takeoff,
                       'e': self.land,
                       '[A': self.forward,
                       '[B': self.backward,
                       '[C': self.right,
                       '[D': self.left,
                       ' ': self.toggleArm,
                    #    'r': self.resetHard,
                       'r' : self.reset}
        
        print("KeyboardControl initialized")

    # This function does nothing and is called for unknown keys
    def justPass(self):
        self.pluto.writerFunc()
    
    # This function resets the Pluto object
    def reset(self):
        # Only reset if it hasn't been called before
        if self.reset_called == False:
            print('reset')
            self.pluto.reset()
            self.reset_called = True
    
    def hardReset(self):
        print('hard reset')
        self.pluto.reset(throttle = True)

    # These functions control the altitude of the Pluto object
    def incAlt(self):
        print("Increase altitude")
        self.pluto.pro.rc.throttle = 1700
        self.pluto.writerFunc()

    def decAlt(self):
        print("Decrease altitude")
        self.pluto.pro.rc.throttle = 1400
        self.pluto.writerFunc()

    # These functions control the yaw of the Pluto object
    def yawLeft(self):
        print("yaw left")
        self.pluto.pro.rc.yaw = 1200
        self.pluto.writerFunc()

    def yawRight(self):
        print("yaw right")
        self.pluto.pro.rc.yaw = 1800
        self.pluto.writerFunc()
        
    # These functions are not implemented
    def takeoff(self):
        print("Takeoff")
    def land(self):
        print("Land")

    # These functions control the pitch and roll of the Pluto object
    def forward(self):
        print("Forward")
        self.pluto.pro.rc.pitch = 1600
        self.pluto.writerFunc()

    def backward(self):
        print("Backward")
        self.pluto.pro.rc.pitch = 1400
        self.pluto.writerFunc()

    def right(self):
        print("Right") 
        self.pluto.pro.rc.roll = 1600
        self.pluto.writerFunc()

    def left(self):
        print("Left")
        self.pluto.pro.rc.roll = 1400
        self.pluto.writerFunc()

    # Toggle the arm state of the drone. Arms or disarms the drone depending on its current state.
    def toggleArm(self):
        print("Toggle arm")
        self.pluto.toggleArm()
    
    # Continuously check for new keystrokes.
    def _key_listener(self):
        while True:
            self.key_logger()

    def __call__(self):
        sleep(self.timeouts)    # Sleep for a short time before checking for new keystrokes
        
        # If a new keystroke has been registered since the last time we checked
        if self.last_key_strock < self.key_logger.reg_time:

            # If the key that was pressed is in our key map
            if self.key_logger.key in self.keyMap:
                self.keyMap[self.key_logger.key]()  # Call the corresponding method from the key map
            
            # If the key that was pressed is not an empty string
            if self.key_logger.key is not '':
                self.reset_called = False   # Reset the reset_called flag
            
            # Update the last_key_strock attribute to the current time
            self.last_key_strock = self.key_logger.reg_time



if __name__ == "__main__":
    args.task = 3
    keyboard_control = KeyboardControl(args)
    while True:
        keyboard_control()