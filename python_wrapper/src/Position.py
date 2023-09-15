class Pose:
    def __init__(self):
        # Initialize x, y, and z attributes to None
        self.x = None
        self.y = None
        self.z = None

    def __str__(self):
        # Return a string representation of the object
        # displaying the values of x, y, and z
        return "x : {}, y : {}, z : {}".format(self.x, self.y, self.z)

class Orientation:
    def __init__(self):
        # Initialize roll, pitch, and yaw attributes to 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class acceleration(Pose):
    def __init__(self):
        # Call superclass's __init__ method to initialize x, y, and z
        super(acceleration, self).__init__()

class gyro(Pose):
    def __init__(self):
        # Call superclass's __init__ method to initialize x, y, and z
        super(gyro, self).__init__()

class mag(Pose):
    def __init__(self):
        # Call superclass's __init__ method to initialize x, y, and z
        super(mag, self).__init__()

class Position:
    def __init__(self):
        # Initialize pose attribute as an instance of the Pose class
        self.pose = Pose()
        # Initialize orientation attribute as an instance of the Orientation class
        self.orientation = Orientation()