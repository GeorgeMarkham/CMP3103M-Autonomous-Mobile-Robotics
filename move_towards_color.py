import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
import cv2
import numpy as np
from numpy import mean, array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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
        #self.wheel_left_vel_sub = rospy.Subscriber("/wheel_vel_left", Float32, self.vel_callback)
        self.vel_pub = rospy.Publisher('/turtlebot_1/cmd_vel', Twist, queue_size=10)

        namedWindow("Robot view", 1)

        self.bridge = CvBridge()
        startWindowThread()

        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,self.img_callback)


    def vel_change(self, data):

        (v, a) = forward_kinematics(data, 0.0)
        print "\n Data:%f" % data
        print "\n\nForward Kinematics\nv = %f,\ta = %f" % (v, a)

        vel_data = Twist()
        vel_data.linear.x = v
        vel_data.angular.x = a

        self.vel_pub.publish(vel_data)
        
    def img_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([ 0,  15,  0])
        upper = np.array([0, 250, 0])
        mask = cv2.inRange(img, lower, upper)

        h,w,_ = img.shape

        M = cv2.moments(mask)

        print(M)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
        else:
            twist = Twist()
            twist.angular.z = 0.5
            self.vel_pub.publish(twist)
        imshow("Robot view", img)


rospy.init_node('robot_wheel_controller')
iv = robot_wheel_controller()
rospy.spin()
