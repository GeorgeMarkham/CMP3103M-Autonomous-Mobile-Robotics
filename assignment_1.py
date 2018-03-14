import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry



class assignment_1:
    def __init__(self):
        rate = rospy.Rate(1)
        self.time = rospy.Time()
        print(self.time)
        #WINDOW TO SHOW WHAT THE ROBOT SEES#
        cv2.namedWindow("Map", 1)

        self.bridge = CvBridge()
        
        cv2.startWindowThread()

        #SET PUBLISHERS#
        self.vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=10)

        #SET SUBSCRIBERS#
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.img_callback)
        self.map_sub = rospy.Subscriber("/turtlebot/move_base/global_costmap/costmap", OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber("/turtlebot/odom", Odometry, self.odom_callback)
        self.point_cloud_sub = rospy.Subscriber("/turtlebot/camera/depth/points", PointCloud2, self.point_cloud_callback)
        self.laser_scan_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_scan_callback)

    def img_callback(self, img_data):
        #img = self.bridge.imgmsg_to_cv2(img_data, 'bgr8')

        #cv2.imshow("Robot View", img)
        pass

    def map_callback(self, map_data):
        np.savetxt('map_data', map_data.data)
        self.origin_x = map_data.info.origin.position.x
        self.origin_y = map_data.info.origin.position.y
        self.origin_z = map_data.info.origin.position.z

        # now = rospy.get_rostime()


        # mv_goal = PoseStamped()
        # mv_goal.header.stamp = self.time
        # mv_goal.header.frame_id = 'map'
        # mv_goal.pose = Pose()
        # mv_goal.pose.position = Point()
        # mv_goal.pose.position.x = 1.0
        # mv_goal.pose.position.y = 1.0
        # mv_goal.pose.position.z = 0.0
        # mv_goal.pose.orientation = Quaternion()
        # mv_goal.pose.orientation.w = 1.0
        # #print(mv_goal)
        # self.goal_pub.publish(mv_goal)

        map_w = map_data.info.width
        map_h = map_data.info.height

        map = np.array(map_data.data).reshape(map_h, map_w)

        np.savetxt('map_data', map)
        print(map_w)
        print(map_h)


        self.width = map_data.info.width
        self.height = map_data.info.height
        self.resolution = map_data.info.resolution
        self.length = len(map_data.data)
        #self.min_line = []

        #creat an mat to load costmap
        costmap_mat = np.zeros((self.height,self.width), dtype = "uint8")

        for i in range(1,self.height):
            for j in range(1,self.width):
                costmap_mat[i-1, j-1] = 255-int(float(map_data.data[(i-1)*self.width+j])/100*255)

        ret,binary_map = cv2.threshold(costmap_mat,127,255,cv2.THRESH_BINARY)

        binary_map = cv2.flip(binary_map, 0)

        cv2.imshow("Map", binary_map)

        np.savetxt('map_data_txt', binary_map)
        np.savez('map_data', binary_map)

    def odom_callback(self, odom_data):
        #print(odom_data.pose)
        pass
    
    def point_cloud_callback(self, point_data):
        #print(point_data.data)
        pass
    def laser_scan_callback(self, laser_data):
        pass

rospy.init_node('assignment_1')
iv = assignment_1()
rospy.spin()


#LASER SCAN OBSTACLE AVOIDANCE#

# scan_angle_min = laser_data.angle_min
# scan_angle_max = laser_data.angle_max

# laser_ranges = laser_data.ranges
# ranges_length = len(laser_ranges)

# should_turn = False
# turn_direction = False # False = Left

# for i in range(ranges_length):
#     if(laser_ranges[i] <= 1.7 and i <= (ranges_length/2)):
#         should_turn = True
#         turn_direction = True
#     elif(laser_ranges[i] <= 1.7 and i <= (ranges_length/2)):
#         should_turn = True
#         pass

# if(should_turn and turn_direction):
#     move_msg = Twist()
#     move_msg.linear.x = 0.0
#     move_msg.angular.z = -0.5

#     self.vel_pub.publish(move_msg)
# elif(should_turn and turn_direction == False):
#     move_msg = Twist()
#     move_msg.linear.x = 0.0
#     move_msg.angular.z = 0.5

#     self.vel_pub.publish(move_msg)
# else:
#     move_msg = Twist()
#     move_msg.linear.x = 0.1
#     move_msg.angular.z = 0.0
#     self.vel_pub.publish(move_msg)
