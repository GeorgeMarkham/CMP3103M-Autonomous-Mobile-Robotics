import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class move_around:
    def __init__(self):
        rate = rospy.Rate(1)
        self.turtlebot = "/turtlebot_1"
        self.image_sub = rospy.Subscriber(self.turtlebot + "/scan",LaserScan,self.scan_callback)

    def scan_callback(self, data):
        print data


rospy.init_node('searcher')
iv = move_around()
rospy.spin()
