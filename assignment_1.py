# CMP3103M - AUTONOMOUS MOBILE ROBOTICS #
# ASSESSMENT 1 #
# GEORGE MARKHAM #
# MAR15561551 #

import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

class assignment:
    def __init__(self):
        rate = rospy.Rate(1)

        self.move_client = actionlib.SimpleActionClient('/turtlebot/move_base/', MoveBaseAction)
        self.move_client.wait_for_server()
        #Initialise point counter to 0 so it goes to the first point first
        self.point_counter = 0

        self.blue_found = False
        self.green_found = False
        self.red_found = False
        self.yellow_found = False

        self.moving = False
        #WINDOW TO SHOW WHAT THE ROBOT SEES#
        cv2.namedWindow("Robot view", 1)
        self.bridge = CvBridge()
        
        cv2.startWindowThread()

        #SET PUBLISHERS#
        self.vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=10)

        #SET SUBSCRIBERS#
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.img_callback)
        self.map_sub = rospy.Subscriber("/turtlebot/move_base/global_costmap/costmap", OccupancyGrid, self.map_callback)
    
    def img_callback(self, img_data):

        # CONVERT THE IMG CALLBACK DATA TO BGR FORMAT, DISPLAY IT AND MAKE IT AVAILABLE TO THE WHOLE CLASS #
        img = self.bridge.imgmsg_to_cv2(img_data, 'bgr8')
        cv2.imshow("Robot view", img)
        self.current_img = img

        #IF THERE'S AN ISSUE THEN MOVE ON TO THE NEXT GOAL #
        if self.move_client.get_state() == actionlib.CommState.LOST or self.move_client.get_state() == actionlib.TerminalState.ABORTED or self.move_client.get_state() == actionlib.CommState.PREEMPTING:
            print("ISSUE at point:", self.point_counter)
            #INCREMENT POINT_COUNTER IF IT'S STILL IN THE LIST
            if self.point_counter + 1 < len(self.area_centers):
                self.point_counter += 1
            #SEND NEXT GOAL
            self.move_to_next_point()



    #MAP CALLBACK#
    def map_callback(self, map_data):
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        map_w = map_data.info.width
        map_h = map_data.info.height
        resolution = map_data.info.resolution



        map = np.array(map_data.data).reshape(map_h, map_w)

        offset = (origin_x, origin_y)


        length = len(map_data.data)
        map = np.zeros((map_h,map_w), dtype = "uint8")
        

        for i in range(1,map_h):
            for j in range(1,map_w):
                map[i-1, j-1] = 255-int(float(map_data.data[(i-1)*map_w+j])/100*255)
        map = cv2.flip(map, 0)
        ret,binary_map = cv2.threshold(map,127,255,cv2.THRESH_BINARY)

        map_centers = self.find_map_centers(binary_map)


        color_map = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

        self.area_centers = []

        for center in map_centers:
            cv2.line(color_map, center, center, (0,0,255), 3)
            self.area_centers.append(self.get_world_pt((center[1], center[0]), offset, resolution))

        self.area_centers.sort(key=lambda x: x[1])

        self.move_to_next_point()

        

        

    def move_to_next_point(self):
        print "MOVE TO NEXT POINT"
        print self.point_counter
        move_to = MoveBaseGoal()
        move_to.target_pose.header.frame_id = "map"
        move_to.target_pose.header.stamp = rospy.Time.now()
        move_to.target_pose.pose.position.x = self.area_centers[self.point_counter][0]
        move_to.target_pose.pose.position.y = self.area_centers[self.point_counter][1]
        move_to.target_pose.pose.orientation.w = 1.0
        self.move_client.send_goal(move_to, done_cb=self.nav_done)

    def nav_done(self, state, res):
        #LOOK FOR A COLOR
        self.turn_left()
        self.turn_right()

        img = self.current_img

        blue = [np.array([15, 0, 0]), np.array([250, 0, 0])]
        green = [np.array([ 0,  15,  0]), np.array([0, 250, 0])]
        red = [np.array([0, 0, 15]), np.array([0, 0, 250])]
        yellow = [np.array([0, 172, 172]), np.array([0, 206, 206])]

        h,w,_ = img.shape
        
        M_blue = cv2.moments(cv2.inRange(img, blue[0], blue[1]))
        M_green = cv2.moments(cv2.inRange(img, green[0], green[1]))
        M_red = cv2.moments(cv2.inRange(img, red[0], red[1]))
        M_yellow = cv2.moments(cv2.inRange(img, yellow[0], yellow[1]))

        #INCREMENT POINT_COUNTER IF IT'S STILL IN THE LIST
        if self.point_counter + 1 < len(self.area_centers):
            self.point_counter += 1

        # Start looking for colors
        print "LOOKING..."
        if M_blue['m00'] > 0 and not self.blue_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found blue!")
            print(M_blue['m00'])

            self.blue_found = True

            cx = int(M_blue['m10']/M_blue['m00'])
            cy = int(M_blue['m01']/M_blue['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            self.move_to_next_point()
        elif M_green['m00'] > 0 and not self.green_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found green!")
            print(M_green['m00'])

            self.green_found = True

            cx = int(M_green['m10']/M_green['m00'])
            cy = int(M_green['m01']/M_green['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            self.move_to_next_point()
        elif M_red['m00'] > 0 and not self.red_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found Red!")
            print(M_red['m00'])

            self.red_found = True

            cx = int(M_red['m10']/M_red['m00'])
            cy = int(M_red['m01']/M_red['m00'])

            print cx
            while M_red < 9000000.0:
                print("Moving to red")
                self.move_client.cancel_goal()
                err = cx - w/2
                twist = Twist()
                twist.linear.x = 0.4
                twist.angular.z = -float(err) / 100
                self.vel_pub.publish(twist)
                rospy.sleep(1)
            self.move_to_next_point()
        elif M_yellow['m00'] > 0 and not self.yellow_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found yellow!")
            print(M_yellow['m00'])

            self.yellow_found = True

            cx = int(M_yellow['m10']/M_yellow['m00'])
            cy = int(M_yellow['m01']/M_yellow['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            self.move_to_next_point()
        else:
            print "FOUND NOTHING"
            self.move_to_next_point()
        
    def turn_left(self):
        turn_count = 0
        while turn_count <= 4:
            rospy.sleep(0.5)
            turn = Twist()
            turn.angular.z = 0.5
            self.vel_pub.publish(turn)
            turn_count += 1
        return
    def turn_right(self):
        turn_count = 0
        while turn_count <= 4:
            rospy.sleep(0.5)
            turn = Twist()
            turn.angular.z = -0.5
            self.vel_pub.publish(turn)
            turn_count += 1
        return
    #FUNCTIONS TO GET MAP COORDINATES OF INTEREST#
    def find_map_centers(self, map):

        #Get 2 copies of the map, one for processing and one to keep as the original copie for checks and such
        original_map = map
        map = map

        h, w = map.shape

        #Process the map to accentuate features
        dilation_kernel = np.ones((8,8), np.uint8)

        map_dilated = cv2.dilate(map, dilation_kernel, iterations=1)

        #Run a canny edge detector to find the outline of objects (specifically looking for walls)
        map_edges = cv2.Canny(map_dilated,100,200)

        minLineLength = 1

        lines = cv2.HoughLinesP(map_edges, rho=1,theta=np.pi/180, threshold=50,lines=np.array([]), minLineLength=minLineLength,maxLineGap=150)

        map_color = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)


        for line in lines[0]:
            pt1_x = line[0]
            pt1_y = line[1]
            pt2_x = line[2]
            pt2_y = line[3]

            pt1 = (pt1_x, pt1_y)
            pt2 = (pt2_x, pt2_y)

            if((pt1_x >= pt2_x - 10) and (pt1_x <= pt2_x + 10) and (pt1_y > pt2_y)):
                pt1 = (pt1_x, 0)
                pt2 = (pt2_x, h)
            if((pt1_y >= pt2_y - 10) and (pt1_y <= pt2_y + 10) and (pt1_x > pt2_x)):
                pt1 = (0, pt1_y)
                pt2 = (w, pt2_y)
            cv2.line(map_color, pt1, pt2, (0,0,0), thickness=5, lineType=8, shift=0)

        map_gray = cv2.cvtColor(map_color, cv2.COLOR_BGR2GRAY)

        ret, binary_map = cv2.threshold(map_gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        opening_kernel = np.ones((3,3), np.uint8)
        map_opening = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, opening_kernel, iterations = 2)
        
        #Split the map into foreground and background

        map_bg = cv2.dilate(map_opening, opening_kernel, iterations = 3)

        ret, map_fg = cv2.threshold(map_opening, 0.2*map_opening.max(), 255, 0)

        map_fg = np.uint8(map_fg)
        map_unknown = cv2.subtract(map_bg, map_fg)

        contours, hierarchy = cv2.findContours(map_unknown.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        center_pts = []
        centers = []

        for contour in contours:
            #peri = cv2.arcLength(contour, True)
            #approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

            if len(contour) >= 4:
                contour_moment = cv2.moments(contour)
                center_pt = (int(contour_moment["m10"]/contour_moment["m00"]), int(contour_moment["m01"]/contour_moment["m00"]))
                centers.append(center_pt)
                #print(contour)

        for center in centers:
            x = int(center[0])
            y = int(center[1])
            #Check the point doesn't lie on an object or too close to an object
            if(x < 12 or y < 22 or x > 200 or y > 240 or original_map[y][x] == 0 or original_map[x-5][y] == 0 or original_map[x+5][y] == 0 or map[x][y-5] == 10 or map[x][y+5] == 0):
                pass
            else:
                center_pts.append(center)
        print len(center_pts)
        return center_pts
    def get_world_pt(self, point, offset, resolution):

        x_map = point[1]
        y_map = point[0]

        x_world = (x_map * resolution) + offset[0]
        y_world = (y_map * resolution) + offset[1]

        world_pt = (x_world, y_world)

        return world_pt

rospy.init_node('assignment')
iv = assignment()
rospy.spin()
