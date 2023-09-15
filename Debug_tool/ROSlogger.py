import rospy
from std_msgs.msg import Float64
import math


class ROSlogger:
    def __init__(self, controller):
        rospy.init_node('ROSlogger' + controller.name, anonymous=True)
        self.controller = controller
        self.z_out_CV = rospy.Publisher(self.controller.name + '/' +'Z_CV', Float64, queue_size=10)
        self.z_out = rospy.Publisher(self.controller.name + '/' +'Z', Float64, queue_size=10)
        self.z_low_pass = rospy.Publisher(self.controller.name + '/' +'Z_lowpass', Float64, queue_size=10)
        self.y_out = rospy.Publisher(self.controller.name + '/' +'Y', Float64, queue_size=10)
        self.x_out = rospy.Publisher(self.controller.name + '/' +'X', Float64, queue_size=10)

        self.x_error = rospy.Publisher(self.controller.name + '/' +'X_error', Float64, queue_size=10)
        self.y_error = rospy.Publisher(self.controller.name + '/' +'Y_error', Float64, queue_size=10)
        self.z_error = rospy.Publisher(self.controller.name + '/' +'Z_error', Float64, queue_size=10)
        self.yaw = rospy.Publisher(self.controller.name + '/' +'Yaw', Float64, queue_size=10)

        self.zero = rospy.Publisher(self.controller.name + '/' +'zero', Float64, queue_size=10)
        self.throttle = rospy.Publisher(self.controller.name + '/' +'RCthrottle', Float64, queue_size = 10)
        self.dist = rospy.Publisher(self.controller.name + '/' +'Distance', Float64, queue_size= 10)
        self.battery = rospy.Publisher(self.controller.name + '/' +'Battery', Float64, queue_size=10)
        self.pitch = rospy.Publisher(self.controller.name + '/' +'RCpitch', Float64, queue_size = 10)
        self.roll = rospy.Publisher(self.controller.name + '/' +'RCroll', Float64, queue_size = 10)
        self.yaw = rospy.Publisher(self.controller.name + '/' +'RCyaw', Float64, queue_size = 10)

        self.dev = rospy.Publisher(self.controller.name + '/' +'Dev', Float64, queue_size = 10)



    def __call__(self):
        # print(self.controller.pos, self.x_position_error)
        self.z_out.publish(self.controller.pos[2])
        self.y_out.publish(self.controller.pos[1])
        self.x_out.publish(self.controller.pos[0])
        self.x_error.publish(self.controller.x_position_error)
        self.y_error.publish(self.controller.y_position_error)
        self.z_error.publish(self.controller.z_position_error)
        self.throttle.publish(self.controller.rcThrottle)
        self.roll.publish(self.controller.rcRoll)
        self.pitch.publish(self.controller.rcPitch)
        self.dist.publish(self.controller.distance)
        self.battery.publish(self.controller. bridge.pro.battery)
        self.dev.publish(self.controller.deviation)
        self.yaw.publish(self.controller.Cyaw)
        self.roll.publish(self.controller.rcYaw)

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