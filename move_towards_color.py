import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
import cv2
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

        mask = cv2.inRange(img, array([0,0,0], 'uint8'), array([0,255,0], 'uint8'))
        img_out = cv2.bitwise_and(img, img, mask=mask)

        img_out_grey = cv2.cvtColor(img_out,cv2.COLOR_BGR2GRAY)

        _,img_out_thresh = cv2.threshold(img_out_grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        contours, hierarchy = cv2.findContours(img_out_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(img, contours, -1, (0,255,0), 3)

        img_height,img_width = img_out_grey.shape
        turn = Twist()
        if(len(contours) > 0 ):

            contour = contours[0][0][0]

            if(contour[0] > ((img_width/2)-15) and  contour[0] < ((img_width/2)+15)):
                
                self.vel_change(data=1.0)

            elif(contour[0] > (img_width/2)):
                turn.angular.z = -0.5
            elif (contour[0] < (img_width/2)):
                turn.angular.z = 0.5

        else:
            turn.angular.z = -0.5

        self.vel_pub.publish(turn)

        imshow("Robot view", img)


rospy.init_node('robot_wheel_controller')
iv = robot_wheel_controller()
rospy.spin()
