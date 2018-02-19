import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

wheel_radius = 0.076
robot_radius = 0.025

# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a) 
    c_r = v + (robot_radius * a) 
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)


class robot_wheel_controller:
    def __init__(self):
        rate = rospy.Rate(1)
        self.wheel_left_vel_sub = rospy.Subscriber("/wheel_vel_left", Float32, self.callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def callback(self, data):

        (v, a) = forward_kinematics(data.data, 0.0)
        print "\n\nForward Kinematics\nv = %f,\ta = %f" % (v, a)

        vel_data = Twist()
        vel_data.linear.x = v
        vel_data.angular.x = a

        self.vel_pub.publish(vel_data)
        

rospy.init_node('robot_wheel_controller')
iv = robot_wheel_controller()
rospy.spin()
