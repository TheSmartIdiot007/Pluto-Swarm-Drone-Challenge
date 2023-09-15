import rospy
from std_msgs.msg import Float64
import math

class ROSlogger:
    def __init__(self, controller):
        self.z_out_CV = rospy.Publisher(controller.name + '/' +'Z_CV', Float64, queue_size=10)
        self.z_out = rospy.Publisher(controller.name + '/' +'Z', Float64, queue_size=10)
        self.z_low_pass = rospy.Publisher(controller.name + '/' +'Z_lowpass', Float64, queue_size=10)
        self.y_out = rospy.Publisher(controller.name + '/' +'Y', Float64, queue_size=10)
        self.x_out = rospy.Publisher(controller.name + '/' +'X', Float64, queue_size=10)

        self.x_error = rospy.Publisher(controller.name + '/' +'X_error', Float64, queue_size=10)
        self.y_error = rospy.Publisher(controller.name + '/' +'Y_error', Float64, queue_size=10)
        self.z_error = rospy.Publisher(controller.name + '/' +'Z_error', Float64, queue_size=10)
        self.yaw = rospy.Publisher(controller.name + '/' +'Yaw', Float64, queue_size=10)

        self.zero = rospy.Publisher(controller.name + '/' +'zero', Float64, queue_size=10)
        self.throttle = rospy.Publisher(controller.name + '/' +'RCthrottle', Float64, queue_size = 10)
        self.dist = rospy.Publisher(controller.name + '/' +'Distance', Float64, queue_size= 10)
        self.battery = rospy.Publisher(controller.name + '/' +'Battery', Float64, queue_size=10)
        self.pitch = rospy.Publisher(controller.name + '/' +'RCpitch', Float64, queue_size = 10)
        self.roll = rospy.Publisher(controller.name + '/' +'RCroll', Float64, queue_size = 10)

        self.dev = rospy.Publisher(controller.name + '/' +'Dev', Float64, queue_size = 10)


    def __call__(self, controller):
        self.z_out.publish(controller.pos[2])
        self.y_out.publish(controller.pos[1])
        self.x_out.publish(controller.pos[0])
        self.x_error.publish(controller.x_position_error)
        self.y_error.publish(controller.y_position_error)
        self.z_error.publish(controller.z_position_error)
        self.throttle.publish(controller.rcThrottle)
        self.roll.publish(controller.rcRoll)
        self.pitch.publish(controller.rcPitch)
        self.dist.publish(controller.distance)
        self.battery.publish(controller. bridge.pro.battery)
        self.dev.publish(controller.deviation)

        self.zero.publish(0)

class testing:
    def __init__(self):
        self.pos = [0, 0, 0]
        self.x_error = 0
        self.y_error = 0
        self.z_error = 0
if __name__ == '__main__':
    test = testing()
    try:
        logger = ROSlogger(test)
    except rospy.ROSInterruptException:
        pass

    while not rospy.is_shutdown():
        test.pos[0] += 0.1
        test.pos[1] += 0.1
        test.pos[2] += 0.5
        test.x_error += 0.1
        test.y_error += 0.1
        test.z_error += math.sin(test.pos[2])
        logger()