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
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid


class assignment:
    def __init__(self):
        rate = rospy.Rate(1)

        # POINT THE ACTIONLIB CLIENT TO THE RIGHT TURTLEBOT MOVE_BASE TOPIC
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
        if self.move_client.get_state() == actionlib.CommState.LOST or self.move_client.get_state() == actionlib.TerminalState.ABORTED:
            print("ISSUE at point:", self.point_counter)
            #CANCLE ALL THE GOALS AND TRY AGAIN#
            self.move_client.cancel_all_goals()
            self.move_to_next_point()


    #MAP CALLBACK#
    def map_callback(self, map_data):
        #Get the origin point in metric space
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        #Get info about the map
        map_w = map_data.info.width
        map_h = map_data.info.height
        resolution = map_data.info.resolution



        map = np.array(map_data.data).reshape(map_h, map_w)

        offset = (origin_x, origin_y)


        length = len(map_data.data)
        #Initalise matrix to hold map
        map = np.zeros((map_h,map_w), dtype = "uint8")
        
        #Fill map matrix with map data
        for r in range(1,map_h):
            for c in range(1,map_w):
                map[r-1, c-1] = 255-int(float(map_data.data[(r-1)*map_w+c])/100*255) #CODE INSPIRED BY user zsbhaha: https://answers.ros.org/question/163801/how-to-correctly-convert-occupancygrid-format-message-to-image/
        map = cv2.flip(map, 0)
        #threshold the map into a binary image
        ret,binary_map = cv2.threshold(map,127,255,cv2.THRESH_BINARY)

        map_centers = self.find_map_centers(binary_map)


        color_map = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

        self.area_centers = []

        #Loop through map points and get metric points from them
        for center in map_centers:
            cv2.line(color_map, center, center, (0,0,255), 3)
            self.area_centers.append(self.get_world_pt((center[1], center[0]), offset, resolution))

        #Sort the map points by the y value so the robot searches from the bottom of the map to the top
        self.area_centers.sort(key=lambda x: x[1])

        #Start searching the map
        self.move_to_next_point()

        

        
    #Function to move to the next goal point#
    def move_to_next_point(self):
        print "MOVE TO NEXT POINT"
        print self.point_counter
        move_to = MoveBaseGoal()
        move_to.target_pose.header.frame_id = "map"
        move_to.target_pose.header.stamp = rospy.Time.now()
        move_to.target_pose.pose.position.x = self.area_centers[self.point_counter][0]
        move_to.target_pose.pose.position.y = self.area_centers[self.point_counter][1]
        move_to.target_pose.pose.orientation.w = 1.0
        #Send the above position to the actionlib move_client with a callback to run when the robot reaches it's goal
        self.move_client.send_goal(move_to, done_cb=self.nav_done)
    
    #Callback function to run when the robot reaches a goal#
    def nav_done(self, state, res):

        #Increment point counter as the last point completed succesfully
        if self.point_counter + 1 <= len(self.area_centers):
            self.point_counter += 1

        #LOOK FOR A COLOR
        self.turn_left()
        self.turn_right()

        #Grab the current image from the camera
        img = self.current_img

        #Set the colours the robot is looking for
        blue = [np.array([15, 0, 0]), np.array([250, 0, 0])]
        green = [np.array([ 0,  15,  0]), np.array([0, 250, 0])]
        red = [np.array([0, 0, 15]), np.array([0, 0, 250])]
        yellow = [np.array([0, 200, 100]), np.array([0, 260, 200])]

        #Get the image height and width so the center point of an object can be found
        h,w,_ = img.shape
        
        #Mask the image to highlight if the colours are found
        M_blue = cv2.moments(cv2.inRange(img, blue[0], blue[1]))
        M_green = cv2.moments(cv2.inRange(img, green[0], green[1]))
        M_red = cv2.moments(cv2.inRange(img, red[0], red[1]))
        M_yellow = cv2.moments(cv2.inRange(img, yellow[0], yellow[1]))

        

        # Start looking for colors
        print "LOOKING..."
        #If any goals are running ensure they are cancelled so they don't interfere with the movement of the robot
        self.move_client.cancel_goals_at_and_before_time(rospy.Time.now())

        #Make sure the robot isn't looking for colours it's already found or that aren't there
        while M_red['m00'] > 0 and not self.red_found:
            #Grab the image and continuously calculate the mask
            img = self.current_img
            M_red = cv2.moments(cv2.inRange(img, red[0], red[1]))
            print("Seen Red!")
            print(M_red['m00'])
            
            #If the robot gets close enough count that as the colour found
            if M_red['m00'] > 1700000:
                self.red_found = True
                print("Found Red!")
                self.move_to_next_point()

            #Get the center of the colour block
            cx = int(M_red['m10']/M_red['m00'])
            cy = int(M_red['m01']/M_red['m00'])

            print("Moving to red")

            #Move towards the colour and keep the colour in the center of the robot's view
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            rospy.sleep(0.1)       

        #Make sure the robot isn't looking for colours it's already found or that aren't there
        while M_blue['m00'] > 0 and not self.blue_found:
            #Grab the image and continuously calculate the mask
            img = self.current_img
            M_blue = cv2.moments(cv2.inRange(img, blue[0], blue[1]))
            print("Seen Blue!")
            print(M_blue['m00'])

            #If the robot gets close enough count that as the colour found
            if M_blue['m00'] > 1700000:
                self.blue_found = True
                print("Found Blue!")
                self.move_to_next_point()

            #Get the center of the colour block
            cx = int(M_blue['m10']/M_blue['m00'])
            cy = int(M_blue['m01']/M_blue['m00'])

            print("Moving to blue")

            #Move towards the colour and keep the colour in the center of the robot's view
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            rospy.sleep(0.1)

        while M_green['m00'] > 0 and not self.green_found:
            #Grab the image and continuously calculate the mask
            img = self.current_img

            M_green = cv2.moments(cv2.inRange(img, green[0], green[1]))
            print("Seen Green!")
            print(M_green['m00'])

            #If the robot gets close enough count that as the colour found
            if M_green['m00'] > 1700000:
                self.green_found = True
                print("Found Green!")
                self.move_to_next_point()

            #Get the center of the colour block
            cx = int(M_green['m10']/M_green['m00'])
            cy = int(M_green['m01']/M_green['m00'])


            print("Moving to green")

            #Move towards the colour and keep the colour in the center of the robot's view
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        while M_yellow['m00'] > 0 and not self.yellow_found:
            #Grab the image and continuously calculate the mask
            img = self.current_img
            M_yellow = cv2.moments(cv2.inRange(img, yellow[0], yellow[1]))
            
            print("Seen Yellow!")
            print(M_yellow['m00'])

            #If the robot gets close enough count that as the colour found
            if M_yellow['m00'] > 2000000:
                self.yellow_found = True
                print("Found Green!")
                self.move_to_next_point()

            #Get the center of the colour block
            cx = int(M_yellow['m10']/M_yellow['m00'])
            cy = int(M_yellow['m01']/M_yellow['m00'])


            print("Moving to yellow")

            #Move towards the colour and keep the colour in the center of the robot's view
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
            rospy.sleep(0.1)

        print "Found Nothing"
        self.move_to_next_point()


    def turn_left(self):
        turn_count = 0
        #Send the command to turn left 4 times
        while turn_count <= 4:
            rospy.sleep(0.5)
            turn = Twist()
            turn.angular.z = 0.5
            self.vel_pub.publish(turn)
            turn_count += 1
        return
    def turn_right(self):
        turn_count = 0
        #Send the command to turn right 4 times
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
        
        #Get any lines in the map
        lines = cv2.HoughLinesP(map_edges, rho=1,theta=np.pi/180, threshold=50,lines=np.array([]), minLineLength=minLineLength,maxLineGap=150)

        map_color = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

        #Extend the lines to seperate sections of the map
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


        #Run an opening on the map
        opening_kernel = np.ones((3,3), np.uint8)
        map_opening = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, opening_kernel, iterations = 2)
        
        #Split the map into foreground and background

        map_bg = cv2.dilate(map_opening, opening_kernel, iterations = 3)

        ret, map_fg = cv2.threshold(map_opening, 0.2*map_opening.max(), 255, 0)

        map_fg = np.uint8(map_fg)
        map_unknown = cv2.subtract(map_bg, map_fg)

        #Segment the map into areas
        contours, hierarchy = cv2.findContours(map_unknown.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        #Initialise lists
        center_pts = []
        centers = []

        for contour in contours:
            # If the amount of points found is at least 4 then an area has been found
            if len(contour) >= 4:
                contour_moment = cv2.moments(contour)
                #Get the x and y center of the area
                center_pt = (int(contour_moment["m10"]/contour_moment["m00"]), int(contour_moment["m01"]/contour_moment["m00"]))
                centers.append(center_pt)

        for center in centers:
            x = int(center[0])
            y = int(center[1])
            #Check the point doesn't lie on an object or too close to an object
            if(x < 12 or y < 22 or x > 200 or y > 240 or original_map[y][x] == 0 or original_map[x-10][y] == 0 or original_map[x+10][y] == 0 or map[x][y-10] == 10 or map[x][y+10] == 0):
                pass
            else:
                center_pts.append(center)

        print "Found " + str(len(center_pts)) + " points of interest"
        
        return center_pts
    
    def get_world_pt(self, point, offset, resolution):

        x_map = point[1]
        y_map = point[0]

        #Resolution is meters per cell so the metric value can be obtained by multiplying a pixel number (the cell) by the resolution and applying an offset (the origin of the map)
        x_world = (x_map * resolution) + offset[0]
        y_world = (y_map * resolution) + offset[1]

        world_pt = (x_world, y_world)

        return world_pt


#Initialise and run the node
rospy.init_node('assignment')
iv = assignment()
rospy.spin()
