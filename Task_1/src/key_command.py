import sys, select, termios, tty  # import necessary modules
from time import time                                   # import time function

class GetKey:
    def __init__(self):
        self.key = ''                                   # initialize key to empty string
        self.settings = termios.tcgetattr(sys.stdin)    # store terminal settings
        self.reg_time = time()                          # store current time

    def getKey(self):
        """Wait for and store key press"""
        tty.setraw(sys.stdin.fileno())                          # set terminal to raw input mode
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)   # listen for input with a timeout of 0.1 seconds
        
        if rlist:                                                # if input was received
            self.key = sys.stdin.read(1)                         # store first character
            if (self.key == '\x1b'):                             # if character is escape key
                self.key = sys.stdin.read(2)                     # store next two characters (e.g. '[A' for up arrow)
            sys.stdin.flush()                                    # flush stdin buffer
        else:
            self.key = ''                                               # no key was pressed within timeout period
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)  # restore terminal settings

        if (self.key == '\x03'):                                        # if user pressed Ctrl-C
            print("Exit")                                               # print message
            exit()                                                      # exit program

    def __call__(self):
        """Update key and reg_time attributes"""
        self.getKey()                                       # get latest key press
        self.reg_time = time()                              # update reg_time with current time

if __name__=="__main__":
    key_logger = GetKey()                               # create GetKey object

    while(1):                                           # infinite loop
        key_logger()                                    # update key and reg_time attributes
        key = key_logger.key                            # store latest key press in variable
        print(key)                                      # print key press

        if (key == 'w'):                                # if 'w' key was pressed
            print("Increase altitude")                 
        if (key == 's'):                                # if 's' key was pressed
            print("Decrease altitude")                 
        if (key == 'a'):                                # if 'a' key was pressed
            print("yaw left")                          
        if (key == 'd'):                                # if 'd' key was pressed
            print("yaw right")                         
        if (key == 'q'):                                # if 'q' key was pressed
            print("Takeoff")                           
        if (key == 'e'):                                # if 'e' key was pressed
            print("Land")                              
        if (key == '[A'):                               # if escape key followed by up arrow was pressed
            print("Forward")                           